param(
    [Parameter(Mandatory=$true)][string]$CubeClt,
    [Parameter(Mandatory=$true)][string]$HostGcc,
    [Parameter(Mandatory=$true)][string]$Python,
    [string]$CsvData = ''
)
$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
$clt=(Resolve-Path -LiteralPath $CubeClt).Path
$gcc=(Resolve-Path -LiteralPath $HostGcc).Path
$pythonExe=(Get-Command $Python -ErrorAction Stop).Source
$cmake=Join-Path $clt 'CMake/bin/cmake.exe'
$ninja=Join-Path $clt 'Ninja/bin/ninja.exe'
$arm=Join-Path $clt 'GNU-tools-for-STM32/bin'
foreach($file in @($cmake,$ninja,(Join-Path $arm 'arm-none-eabi-gcc.exe'),$gcc)) {
    if(!(Test-Path -LiteralPath $file -PathType Leaf)) { throw "Missing tool: $file" }
}
$local=Join-Path $root '.local'
New-Item -ItemType Directory -Force -Path $local | Out-Null
$config=@{CubeClt=$clt; HostGcc=$gcc; Python=$pythonExe; CsvData=$CsvData}
$utf8=[Text.UTF8Encoding]::new($false)
[IO.File]::WriteAllText((Join-Path $local 'tools.json'),($config|ConvertTo-Json),$utf8)
$configure=@(); $build=@()
foreach($name in @('Debug','Release','Diagnostics')) {
    $configure+=@{name="$name-local"; displayName="$name (STM32CubeCLT)"; inherits=$name;
        environment=@{};
        cacheVariables=@{CMAKE_MAKE_PROGRAM=$ninja}}
    # Keep the CMake environment macro literal, not PowerShell interpolation.
    $configure[-1].environment.PATH="$arm;$(Split-Path $ninja -Parent);$(Split-Path $cmake -Parent);" + '$penv{PATH}'
    $build+=@{name="$name-local"; configurePreset="$name-local"; jobs=1}
}
$presets=@{version=3; configurePresets=$configure; buildPresets=$build}
[IO.File]::WriteAllText((Join-Path $root 'CMakeUserPresets.json'),($presets|ConvertTo-Json -Depth 8),$utf8)
Write-Host 'Local tool paths configured. Select Debug-local, Release-local or Diagnostics-local.'
