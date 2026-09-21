$ErrorActionPreference='Stop'
$repo=Split-Path $PSScriptRoot -Parent
$base='34d57c2a93b57979fecb929067e49df47c17c14d'
$source=(git -C $repo show "${base}:Core/Src/cc1101.c")-join"`n"
$match=[regex]::Match($source,'(?m)^void CC1101SendHandler\(void\)')
if(!$match.Success) { throw 'Protocol function not found in baseline' }
$start=$match.Index
$end=$source.IndexOf('/*',$source.IndexOf('memset(&cc1101, 0, sizeof(cc1101));',$start))
# The next top-level documentation block starts after the function.
$body=$source.Substring($start,$end-$start)
$template=[IO.File]::ReadAllText((Join-Path $repo 'Tests/legacy/protocol-template.txt'))
$output=Join-Path $repo '.local/protocol-regeneration'
New-Item -ItemType Directory -Force -Path $output | Out-Null
[IO.File]::WriteAllText((Join-Path $output 'protocol.c'),$template.Replace('/* BASE_FUNCTION */',$body),[Text.UTF8Encoding]::new($false))
$table=$source.Substring($source.IndexOf('static const uint8_t CC1101InitData'))
$table=$table.Substring(0,$table.IndexOf('};')+2)
$values=[regex]::Matches($table,'\{CC1101_\w+,\s*(0x[0-9A-Fa-f]+)\}') | ForEach-Object { $_.Groups[1].Value }
if($values.Count-ne47) { throw 'RF configuration extraction failed' }
[IO.File]::WriteAllText((Join-Path $output 'rf_config.h'),('/* Exact 47-byte register table from '+$base+' */'+"`nstatic const uint8_t gold[47]={"+($values-join',')+"};`n"),[Text.UTF8Encoding]::new($false))
Write-Host "Reference generated in $output; review before replacing frozen tests."
