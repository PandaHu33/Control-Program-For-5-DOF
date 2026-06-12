param(
    [string]$DeviceId = "",
    [int]$Port = 8070
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectRoot = Split-Path -Parent $Root
Set-Location $ProjectRoot

function Write-Info($Text) { Write-Host "[INFO] $Text" -ForegroundColor Cyan }
function Write-Ok($Text) { Write-Host "[OK] $Text" -ForegroundColor Green }
function Write-Warn($Text) { Write-Host "[WARN] $Text" -ForegroundColor Yellow }

function Find-Adb {
    if ($env:ADB_EXE -and (Test-Path $env:ADB_EXE)) {
        return $env:ADB_EXE
    }

    $cmd = Get-Command adb -ErrorAction SilentlyContinue
    if ($cmd) {
        return $cmd.Source
    }

    $candidates = @(
        "$env:LOCALAPPDATA\Android\Sdk\platform-tools\adb.exe",
        "$env:ANDROID_HOME\platform-tools\adb.exe",
        "$env:ANDROID_SDK_ROOT\platform-tools\adb.exe",
        "C:\Program Files\Unity\Hub\Editor\2022.3.15f1c1\Editor\Data\PlaybackEngines\AndroidPlayer\SDK\platform-tools\adb.exe"
    )

    foreach ($candidate in $candidates) {
        if ($candidate -and (Test-Path $candidate)) {
            return $candidate
        }
    }

    return $null
}

function Get-AdbDeviceId($AdbPath, $PreferredDeviceId) {
    if ($PreferredDeviceId) {
        return $PreferredDeviceId
    }

    $deviceLines = & $AdbPath devices | Where-Object { $_ -match "^\S+\s+device$" }
    $devices = @($deviceLines | ForEach-Object { ($_ -split "\s+")[0] })

    if ($devices.Count -eq 1) {
        return $devices[0]
    }

    if ($devices.Count -gt 1) {
        throw "Multiple ADB devices found. Start with: run_vr_static_server.bat DEVICE_ID"
    }

    throw "No ADB device is connected or authorized."
}

function Invoke-AdbReverse($AdbPath, $ResolvedDeviceId, $ReversePort) {
    Write-Info "adb reverse tcp:$ReversePort tcp:$ReversePort"
    & $AdbPath -s $ResolvedDeviceId reverse "tcp:$ReversePort" "tcp:$ReversePort"
    if ($LASTEXITCODE -ne 0) {
        throw "adb reverse failed on port $ReversePort"
    }
}

Write-Host ""
Write-Host "========================================"
Write-Host " UEM VR H5 Static Server + ADB Reverse"
Write-Host "========================================"
Write-Host ""

$adb = Find-Adb
if (-not $adb) {
    throw "adb.exe was not found. Set ADB_EXE or install Android platform-tools."
}

Write-Info "Using adb: $adb"
$resolvedDeviceId = Get-AdbDeviceId $adb $DeviceId
Write-Info "Using device: $resolvedDeviceId"

$reversePorts = @(8070, 8080, 8090, 8091, 8081, 5005)
foreach ($reversePort in $reversePorts) {
    Invoke-AdbReverse $adb $resolvedDeviceId $reversePort
}

Write-Ok "ADB reverse ports are ready."
& $adb -s $resolvedDeviceId reverse --list

Write-Host ""
Write-Info "Serving H5 files for Unity WebView."
Write-Info "URL inside headset/Unity WebView: http://127.0.0.1:$Port/control_ui/index.html"
Write-Warn "This only starts a static file server. It does not start Jetson, bridge, hand tracker, or robot processes."
Write-Warn "Press Ctrl+C to stop."
Write-Host ""

python -m http.server $Port --bind 127.0.0.1
