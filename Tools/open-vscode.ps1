$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
$config=Get-Content -Raw (Join-Path $root '.local/tools.json') | ConvertFrom-Json
$env:STM32_CUBE_CLT=$config.CubeClt
& code --new-window $root
if($LASTEXITCODE -ne 0) { throw 'VS Code did not open the workspace.' }
