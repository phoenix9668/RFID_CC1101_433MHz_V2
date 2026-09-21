param([switch]$Install, [switch]$RequireEnvironment)
$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
$config=Get-Content -Raw (Join-Path $root '.local/tools.json') | ConvertFrom-Json
if($RequireEnvironment -and $env:STM32_CUBE_CLT -ine $config.CubeClt) {
    throw 'Open this workspace with Tools/open-vscode.ps1 so debugging uses the verified CLT.'
}
$program=(Resolve-Path (Join-Path $config.CubeClt 'STLink-gdb-server/bin/ST-LINK_gdbserver.exe')).Path
$name='RFID-V38-STLink-Block-Remote'
$ranges=@('0.0.0.0-126.255.255.255','128.0.0.0-255.255.255.255','::2-ffff:ffff:ffff:ffff:ffff:ffff:ffff:ffff')
if($Install) {
    $admin=[Security.Principal.WindowsPrincipal]::new([Security.Principal.WindowsIdentity]::GetCurrent())
    if(!$admin.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
        throw 'Run this script with -Install in an administrator PowerShell to restrict remote debug access.'
    }
    # A deny-only rule scoped to this executable; no existing rules are removed.
    $existing=Get-NetFirewallRule -Name $name -ErrorAction SilentlyContinue
    if(!$existing) {
        New-NetFirewallRule -Name $name -DisplayName $name -Direction Inbound -Action Block -Enabled True -Profile Any -Program $program -Protocol TCP -RemoteAddress $ranges | Out-Null
    }
}
$profiles=@(Get-NetFirewallProfile -PolicyStore ActiveStore)
if($profiles.Count -ne 3 -or @($profiles | Where-Object { $_.Enabled -ne 'True' }).Count) {
    throw 'All Windows Firewall profiles must be enabled before ST debugging.'
}
$rule=Get-NetFirewallRule -PolicyStore ActiveStore -Name $name -ErrorAction SilentlyContinue
if(!$rule -or $rule.Enabled -ne 'True' -or $rule.Direction -ne 'Inbound' -or $rule.Action -ne 'Block' -or $rule.Profile -ne 'Any') {
    throw 'Remote-debug protection is not installed. Run Tools/debug-network.ps1 -Install as administrator.'
}
$app=$rule | Get-NetFirewallApplicationFilter
$address=$rule | Get-NetFirewallAddressFilter
$port=$rule | Get-NetFirewallPortFilter
$expected=$ranges | Sort-Object
$actual=@($address.RemoteAddress) | Sort-Object
if($app.Program -ine $program -or $port.Protocol -ne 'TCP' -or $port.LocalPort -ne 'Any' -or
   (Compare-Object $expected $actual)) {
    throw 'Remote-debug firewall rule does not match the configured ST server and address ranges.'
}
Write-Output 'Verified: ST server remote inbound TCP is blocked; localhost debugging remains available.'
