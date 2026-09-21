param([string]$Base = '34d57c2a93b57979fecb929067e49df47c17c14d',
      [string]$Output = (Join-Path (Split-Path $PSScriptRoot -Parent) '.local/reference-regeneration'))
$ErrorActionPreference = 'Stop'
$repo = Split-Path $PSScriptRoot -Parent
$root = [IO.Path]::GetFullPath($Output)
foreach($directory in @('Tests/legacy','App/Src')) {
    New-Item -ItemType Directory -Force -Path (Join-Path $root $directory) | Out-Null
}
$utf8 = [System.Text.UTF8Encoding]::new($false)
$source = (git -C $repo show "${Base}:Core/Src/adxl362.c") -join "`n"
$header = (git -C $repo show "${Base}:Core/Inc/adxl362.h") -join "`n"
[IO.File]::WriteAllText((Join-Path $root 'Tests/legacy/adxl362.c'), $source + "`n", $utf8)
[IO.File]::WriteAllText((Join-Path $root 'Tests/legacy/adxl362.h'), $header + "`n", $utf8)
$branch = $source.Substring($source.IndexOf('void ADXL362FifoProcess(void)'))
$branch = $branch.Substring($branch.IndexOf('#else'))
$start = $branch.IndexOf('    // 3.Average Calc')
$loop = $branch.IndexOf('    for (uint16_t i = 0;', $start)
$begin = $branch.IndexOf('{', $loop)
$depth = 1
$end = $begin + 1
while ($depth -gt 0) {
    if ($branch[$end] -eq '{') { $depth++ }
    if ($branch[$end] -eq '}') { $depth-- }
    $end++
}
$body = $branch.Substring($begin + 1, $end - $begin - 2)
$body = $body.Replace('three_axis_info[i]', 'sample')
$body = $body.Replace('if (((i + 1) / 25) != 0 && ((i + 1) % 25) == 0)', 'if (++pending_samples == 25)')
$body = [regex]::Replace($body, '(?m)^.*rfid_printf\(.*\);\s*$', '')
$body = [regex]::Replace($body, '(?s)\s*// print memory_array\s*for .*?\n            }', '')
$body = [regex]::Replace($body, '(?m)^\s*//.*$', '')
$body = [regex]::Replace($body, '(?s)            action_classify_array\[.*?action_classify.other\+\+;', '            *result = (uint8_t)memory_array[memory_index_o][0];')
$body = $body.Replace('            z_min_val = 2000;', "            z_min_val = 2000;`n            pending_samples = 0;`n            return true;")
$declStart = $source.IndexOf('axis_info_int16_t diff_three_axis_info')
$declEnd = $source.IndexOf('uint8_t fifo[')
$decl = $source.Substring($declStart, $declEnd - $declStart)
$decl = [regex]::Replace($decl, '(?m)^action_classify_t.*\n|^uint8_t action_classify_array.*\n', '')
$decl = [regex]::Replace($decl, '(?m)^(axis_info_|threshold_judge_t|uint8_t|int32_t|int16_t|uint16_t)', 'static $1')
$prefix = @'
/* Mechanically extracted from 34d57c2. Thresholds and integer arithmetic preserved. */
#include "behavior.h"
#include <stdlib.h>
#include <string.h>
#define _DIFF_CNT 2
#define _MEM_ROWS 18
#define _MEM_COLS 4
typedef accel_sample_t axis_info_int16_t;
typedef struct { int32_t x, y, z; } axis_info_int32_t;
typedef struct { uint16_t low, normal, abovenormal, high; } threshold_judge_t;
'@
$suffix = @'
void behavior_reset(void)
{
    memset(diff_three_axis_info, 0, sizeof(diff_three_axis_info));
    memset(&three_axis_average_info, 0, sizeof(three_axis_average_info));
    memset(&sum_info, 0, sizeof(sum_info));
    memset(&threshold_judge, 0, sizeof(threshold_judge));
    memset(memory_array, 0, sizeof(memory_array));
    memory_index = memory_index_o = pending_samples = 0;
    x_max_val = y_max_val = z_max_val = -2000;
    x_min_val = y_min_val = z_min_val = 2000;
}
'@
$output = $prefix + "`n" + $decl + "`nstatic uint8_t pending_samples;`nbool behavior_push(accel_sample_t sample, uint8_t *result)`n{`n" + $body + "`n    return false;`n}`n" + $suffix + "`n"
[IO.File]::WriteAllText((Join-Path $root 'App/Src/behavior.c'), $output, $utf8)
