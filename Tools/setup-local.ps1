param(
    [Parameter(Mandatory=$true)][string]$CubeClt,
    [Parameter(Mandatory=$true)][string]$HostGcc,
    [Parameter(Mandatory=$true)][string]$Python,
    [string]$CsvData = '',
    # STM32Cube bundle root, e.g. "$env:LOCALAPPDATA\stm32cube\bundles". When given, the
    # firmware build uses the versions pinned in .settings/bundles.store.json, not CubeCLT's.
    [string]$Bundles = ''
)
$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
$clt=(Resolve-Path -LiteralPath $CubeClt).Path
$gcc=(Resolve-Path -LiteralPath $HostGcc).Path
$pythonExe=(Get-Command $Python -ErrorAction Stop).Source
if($Bundles) {
    $bundleRoot=(Resolve-Path -LiteralPath $Bundles).Path
    $store=Get-Content -Raw -LiteralPath (Join-Path $root '.settings/bundles.store.json') | ConvertFrom-Json
    $pinned=@{}
    foreach($bundle in $store.bundles) { $pinned[$bundle.name]=$bundle.version }
    foreach($name in @('gnu-tools-for-stm32','cmake','ninja')) {
        if(!$pinned[$name]) { throw "bundles.store.json does not pin $name" }
    }
    $arm=Join-Path $bundleRoot "gnu-tools-for-stm32/$($pinned['gnu-tools-for-stm32'])/bin"
    $cmake=Join-Path $bundleRoot "cmake/$($pinned['cmake'])/bin/cmake.exe"
    $ninja=Join-Path $bundleRoot "ninja/$($pinned['ninja'])/bin/ninja.exe"
    $label="GCC $($pinned['gnu-tools-for-stm32']) bundle"
} else {
    $cmake=Join-Path $clt 'CMake/bin/cmake.exe'
    $ninja=Join-Path $clt 'Ninja/bin/ninja.exe'
    $arm=Join-Path $clt 'GNU-tools-for-STM32/bin'
    $label='STM32CubeCLT'
}
foreach($file in @($cmake,$ninja,(Join-Path $arm 'arm-none-eabi-gcc.exe'),$gcc)) {
    if(!(Test-Path -LiteralPath $file -PathType Leaf)) { throw "Missing tool: $file" }
}
$armVersion=(& (Join-Path $arm 'arm-none-eabi-gcc.exe') --version)[0]
$local=Join-Path $root '.local'
New-Item -ItemType Directory -Force -Path $local | Out-Null
# CubeClt stays required: backup and debug scripts use its programmer and GDB server.
$config=@{CubeClt=$clt; HostGcc=$gcc; Python=$pythonExe; CsvData=$CsvData;
    ArmBin=$arm; CMake=$cmake; Ninja=$ninja; ArmGccVersion=$armVersion}
$utf8=[Text.UTF8Encoding]::new($false)
[IO.File]::WriteAllText((Join-Path $local 'tools.json'),($config|ConvertTo-Json),$utf8)
$configure=@(); $build=@()
foreach($name in @('Debug','Release','Diagnostics')) {
    $configure+=@{name="$name-local"; displayName="$name ($label)"; inherits=$name;
        environment=@{};
        cacheVariables=@{CMAKE_MAKE_PROGRAM=$ninja}}
    # Keep the CMake environment macro literal, not PowerShell interpolation.
    $configure[-1].environment.PATH="$arm;$(Split-Path $ninja -Parent);$(Split-Path $cmake -Parent);" + '$penv{PATH}'
    $build+=@{name="$name-local"; configurePreset="$name-local"; jobs=1}
}
$presets=@{version=3; configurePresets=$configure; buildPresets=$build}
[IO.File]::WriteAllText((Join-Path $root 'CMakeUserPresets.json'),($presets|ConvertTo-Json -Depth 8),$utf8)
Write-Host "Local tool paths configured ($armVersion). Select Debug-local, Release-local or Diagnostics-local."
