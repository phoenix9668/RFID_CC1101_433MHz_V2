param([Parameter(Mandatory=$true)][string]$Port,[int]$Seconds=90)
$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
$dir=Join-Path $root '.local/captures'
New-Item -ItemType Directory -Force -Path $dir | Out-Null
$path=Join-Path $dir ((Get-Date -Format 'yyyyMMdd-HHmmss')+'-'+$Port+'.txt')
$serial=[IO.Ports.SerialPort]::new($Port,115200,[IO.Ports.Parity]::None,8,[IO.Ports.StopBits]::One)
$serial.DtrEnable=$false; $serial.RtsEnable=$false; $serial.ReadTimeout=200
$writer=[IO.StreamWriter]::new($path,$false,[Text.UTF8Encoding]::new($false))
$timed=[IO.StreamWriter]::new(($path+'.jsonl'),$false,[Text.UTF8Encoding]::new($false))
try {
    $serial.Open()
    Write-Host "Capturing $Port for $Seconds seconds: $path"
    $end=[DateTime]::UtcNow.AddSeconds($Seconds)
    while([DateTime]::UtcNow-lt$end) {
        $text=$serial.ReadExisting()
        if($text) {
            $writer.Write($text); $writer.Flush(); Write-Host -NoNewline $text
            $timed.WriteLine((@{utc=[DateTime]::UtcNow.ToString('o');text=$text}|ConvertTo-Json -Compress))
            $timed.Flush()
        }
        Start-Sleep -Milliseconds 100
    }
} finally { $serial.Close(); $serial.Dispose(); $writer.Dispose(); $timed.Dispose() }
