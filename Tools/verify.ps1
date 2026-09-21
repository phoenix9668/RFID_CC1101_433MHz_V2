param([switch]$HostOnly, [string]$BuildRoot='build/codex-verify')
$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
$config=Get-Content -Raw -LiteralPath (Join-Path $root '.local/tools.json') | ConvertFrom-Json
$cmake=Join-Path $config.CubeClt 'CMake/bin/cmake.exe'
$ctest=Join-Path $config.CubeClt 'CMake/bin/ctest.exe'
$env:PATH="$(Join-Path $config.CubeClt 'GNU-tools-for-STM32/bin');$(Join-Path $config.CubeClt 'Ninja/bin');$(Split-Path $config.HostGcc -Parent);$env:PATH"
function Run([string]$Program,[string[]]$Arguments) {
    & $Program @Arguments
    if($LASTEXITCODE -ne 0) { throw "$Program failed with exit code $LASTEXITCODE" }
}
Push-Location $root
try {
    Run $cmake @('--version')
    Run (Join-Path $config.CubeClt 'GNU-tools-for-STM32/bin/arm-none-eabi-gcc.exe') @('--version')
    if(!$HostOnly) {
        foreach($preset in @('Debug','Release','Diagnostics')) {
            $dir="$BuildRoot/$preset"
            Run $cmake @('--preset',"$preset-local",'-B',$dir)
            Run $cmake @('--build',$dir,'--parallel','1')
        }
    }
    $hostDir="$BuildRoot/host"
    Run $cmake @('-S','.','-B',$hostDir,'-G','Ninja','-DRFID_HOST_TESTS=ON',"-DCMAKE_C_COMPILER=$($config.HostGcc.Replace('\','/'))")
    Run $cmake @('--build',$hostDir,'--parallel','1')
    Run $ctest @('--test-dir',$hostDir,'--output-on-failure')
    if($config.CsvData) {
        Run $config.Python @('Tools/behavior-csv.py','--exe',"$hostDir/Tests/behavior_regression.exe",'--data',$config.CsvData,'--out',"$BuildRoot/csv-results")
    } else { Write-Warning 'CSV fixtures not configured; synthetic regression still ran.' }
} finally { Pop-Location }
