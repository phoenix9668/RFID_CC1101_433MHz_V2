param([ValidateSet('Debug','Release','Diagnostics')][string]$Preset='Debug')
$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
$config=Get-Content -Raw -LiteralPath (Join-Path $root '.local/tools.json') | ConvertFrom-Json
$cmake=Join-Path $config.CubeClt 'CMake/bin/cmake.exe'
Push-Location $root
try {
    & $cmake --preset "$Preset-local"
    if($LASTEXITCODE) { throw 'Configure failed' }
    & $cmake --build --preset "$Preset-local"
    if($LASTEXITCODE) { throw 'Build failed' }
} finally { Pop-Location }
