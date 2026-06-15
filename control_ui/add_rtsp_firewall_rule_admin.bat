@echo off
setlocal

set "RULE_NAME=UEM RTSP Camera MediaMTX 8554"
set "SCRIPT=%TEMP%\uem_add_rtsp_firewall_rule.ps1"

> "%SCRIPT%" echo $ErrorActionPreference = 'Stop'
>> "%SCRIPT%" echo $ruleName = 'UEM RTSP Camera MediaMTX 8554'
>> "%SCRIPT%" echo $existing = Get-NetFirewallRule -DisplayName $ruleName -ErrorAction SilentlyContinue
>> "%SCRIPT%" echo if ($existing) {
>> "%SCRIPT%" echo     Set-NetFirewallRule -DisplayName $ruleName -Enabled True -Direction Inbound -Action Allow
>> "%SCRIPT%" echo     $existing ^| Get-NetFirewallPortFilter ^| Set-NetFirewallPortFilter -Protocol TCP -LocalPort 8554
>> "%SCRIPT%" echo } else {
>> "%SCRIPT%" echo     New-NetFirewallRule -DisplayName $ruleName -Direction Inbound -Action Allow -Protocol TCP -LocalPort 8554 ^| Out-Null
>> "%SCRIPT%" echo }
>> "%SCRIPT%" echo Write-Host 'Firewall rule is ready: TCP 8554 inbound allowed.' -ForegroundColor Green
>> "%SCRIPT%" echo Read-Host 'Press Enter to close'

powershell -NoProfile -ExecutionPolicy Bypass -Command "Start-Process powershell -Verb RunAs -ArgumentList '-NoProfile -ExecutionPolicy Bypass -File ""%SCRIPT%""'"
