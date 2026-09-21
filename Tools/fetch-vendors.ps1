param([string]$Destination = (Split-Path $PSScriptRoot -Parent))
$ErrorActionPreference = 'Stop'
$adi = 'https://raw.githubusercontent.com/analogdevicesinc/no-OS/0c4ef5cd278555ba4d426b35f87f30102460361f'
$files = @{
    'Drivers/ADI/adxl362.c' = "$adi/drivers/accel/adxl362/adxl362.c"
    'Drivers/ADI/adxl362.h' = "$adi/drivers/accel/adxl362/adxl362.h"
    'Drivers/PCG/pcg_basic.c' = 'https://raw.githubusercontent.com/imneme/pcg-c-basic/bc39cd76ac3d541e618606bcc6e1e5ba5e5e6aa3/pcg_basic.c'
    'Drivers/PCG/pcg_basic.h' = 'https://raw.githubusercontent.com/imneme/pcg-c-basic/bc39cd76ac3d541e618606bcc6e1e5ba5e5e6aa3/pcg_basic.h'
    'Drivers/PCG/LICENSE.txt' = 'https://raw.githubusercontent.com/imneme/pcg-c-basic/bc39cd76ac3d541e618606bcc6e1e5ba5e5e6aa3/LICENSE.txt'
    'Core/Startup/startup_stm32l051xx.s' = 'https://raw.githubusercontent.com/STMicroelectronics/cmsis-device-l0/9d7674c4560773dc63ed988fa69959296a9c7935/Source/Templates/gcc/startup_stm32l051xx.s'
}
foreach ($entry in $files.GetEnumerator()) {
    $path = Join-Path $Destination $entry.Key
    New-Item -ItemType Directory -Force -Path (Split-Path $path -Parent) | Out-Null
    Invoke-WebRequest -Uri $entry.Value -OutFile $path
    Get-FileHash -Algorithm SHA256 -LiteralPath $path | Select-Object Hash, Path
}
