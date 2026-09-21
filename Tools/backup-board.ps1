param([Parameter(Mandatory=$true)][ValidatePattern('^[A-Za-z0-9_-]+$')][string]$ProbeSerial,
      [ValidateSet('HOTPLUG','UR')][string]$Mode='HOTPLUG')
$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
$config=Get-Content -Raw -LiteralPath (Join-Path $root '.local/tools.json') | ConvertFrom-Json
$cli=Join-Path $config.CubeClt 'STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe'
$dir=Join-Path $root ('.local/backups/'+(Get-Date -Format 'yyyyMMdd-HHmmss')+'-'+$ProbeSerial)
if(Test-Path -LiteralPath $dir) { throw 'Refusing to overwrite an existing backup' }
New-Item -ItemType Directory -Path $dir | Out-Null
$flash=Join-Path $dir 'flash.bin'
$eeprom=Join-Path $dir 'eeprom.bin'
$check=Join-Path $dir 'eeprom-check.bin'
$options=Join-Path $dir 'options.bin'
# Upload/display only: never erase, download or unprotect. UR explicitly resets
# and holds the target while connected; the CLI may resume it on disconnect.
# Connect NRST and stop production operation before use.
& $cli -c port=SWD "sn=$ProbeSerial" "mode=$Mode" freq=100 -u 0x08000000 0x10000 $flash -u 0x08080000 0x800 $eeprom -ob displ -u 0x1ff80000 20 $options -u 0x08080000 0x800 $check 2>&1 | Tee-Object -FilePath (Join-Path $dir 'programmer.txt')
if($LASTEXITCODE) { throw 'Read-only backup failed. Do not change RDP or flash the board.' }
foreach($f in @($flash,$eeprom,$check,$options)) { if(!(Test-Path -LiteralPath $f)) { throw "Missing backup: $f" } }
if((Get-Item -LiteralPath $flash).Length-ne65536 -or (Get-Item -LiteralPath $eeprom).Length-ne2048 -or (Get-Item -LiteralPath $options).Length-ne20 -or
   (Get-FileHash -LiteralPath $eeprom).Hash-ne(Get-FileHash -LiteralPath $check).Hash) {
    throw 'Incomplete or changing EEPROM snapshot; keep files and retry read-only backup.'
}
$bytes=[IO.File]::ReadAllBytes($eeprom)
$id=@($bytes[3],$bytes[2],$bytes[1],$bytes[0],$bytes[7],$bytes[6]) | ForEach-Object { $_.ToString('X2') }
$manifest=@{ProbeSerial=$ProbeSerial; DeviceId=($id-join''); ReadOnly=$true; ConnectionMode=$Mode; Complete=$true;
    FlashSHA256=(Get-FileHash -LiteralPath $flash).Hash;
    EepromSHA256=(Get-FileHash -LiteralPath $eeprom).Hash;
    OptionsSHA256=(Get-FileHash -LiteralPath $options).Hash;
    Options='programmer.txt'; CreatedUtc=[DateTime]::UtcNow.ToString('o')}
[IO.File]::WriteAllText((Join-Path $dir 'manifest.json'),($manifest|ConvertTo-Json),[Text.UTF8Encoding]::new($false))
Write-Host "Verified read-only backup: $dir"
