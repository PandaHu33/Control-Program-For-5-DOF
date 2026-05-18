$ErrorActionPreference = "Stop"

$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $Root

$utf8 = [System.Text.UTF8Encoding]::new()
[Console]::OutputEncoding = $utf8
$OutputEncoding = $utf8

$ApiBase = "http://127.0.0.1:8090"
$UiPath = Join-Path $Root "index.html"
$LogDir = Join-Path $Root "logs"
$BackendOut = Join-Path $LogDir "bridge_stdout.log"
$BackendErr = Join-Path $LogDir "bridge_stderr.log"
$script:StartedSystem = $false
$script:StopObserved = $false
$script:BackendProcess = $null

New-Item -ItemType Directory -Force -Path $LogDir | Out-Null

function Write-Info($Text) {
    Write-Host "[INFO] $Text" -ForegroundColor Cyan
}

function Write-Ok($Text) {
    Write-Host "[OK] $Text" -ForegroundColor Green
}

function Write-Warn($Text) {
    Write-Host "[WARN] $Text" -ForegroundColor Yellow
}

function Write-Fail($Text) {
    Write-Host "[FAIL] $Text" -ForegroundColor Red
}

function Get-ApiStatus {
    try {
        return Invoke-RestMethod -Uri "$ApiBase/api/status" -Method Get -TimeoutSec 2
    } catch {
        return $null
    }
}

function Wait-BackendReady($Seconds) {
    $deadline = (Get-Date).AddSeconds($Seconds)
    while ((Get-Date) -lt $deadline) {
        $status = Get-ApiStatus
        if ($null -ne $status -and $status.ok) {
            return $true
        }
        Start-Sleep -Milliseconds 500
    }
    return $false
}

function Stop-ExistingBackend {
    $ownerPids = @()
    try {
        $ownerPids += Get-NetTCPConnection -LocalPort 8080,8090 -State Listen -ErrorAction SilentlyContinue | Select-Object -ExpandProperty OwningProcess
    } catch {}
    try {
        $ownerPids += Get-NetUDPEndpoint -LocalPort 14550 -ErrorAction SilentlyContinue | Select-Object -ExpandProperty OwningProcess
    } catch {}

    $ownerPids = $ownerPids | Where-Object { $_ -and $_ -gt 0 } | Sort-Object -Unique
    foreach ($ownerPid in $ownerPids) {
        try {
            $proc = Get-Process -Id $ownerPid -ErrorAction Stop
            if ($proc.ProcessName -in @("python", "pythonw", "bridge")) {
                Write-Info ("Stopping old local backend pid={0} process={1}" -f $ownerPid, $proc.ProcessName)
                Stop-Process -Id $ownerPid -Force
            } else {
                Write-Warn ("Port is occupied by pid={0} process={1}; not stopping it automatically." -f $ownerPid, $proc.ProcessName)
            }
        } catch {}
    }

    Start-Sleep -Milliseconds 800
}

function Start-Backend {
    Stop-ExistingBackend

    Write-Info "Starting local backend bridge..."
    Remove-Item -LiteralPath $BackendOut, $BackendErr -Force -ErrorAction SilentlyContinue

    $bridgeExe = Join-Path $Root "bridge\bridge.exe"
    if (-not (Test-Path $bridgeExe)) {
        $bridgeExe = Join-Path $Root "bridge.exe"
    }
    if (Test-Path $bridgeExe) {
        $script:BackendProcess = Start-Process -FilePath $bridgeExe `
            -WorkingDirectory $Root `
            -WindowStyle Hidden `
            -RedirectStandardOutput $BackendOut `
            -RedirectStandardError $BackendErr `
            -PassThru
    } else {
        $python = Get-Command python -ErrorAction SilentlyContinue
        if ($python) {
            Write-Warn "bridge.exe was not found. Falling back to python bridge.py."
            $script:BackendProcess = Start-Process -FilePath $python.Source `
                -ArgumentList "bridge.py" `
                -WorkingDirectory $Root `
                -WindowStyle Hidden `
                -RedirectStandardOutput $BackendOut `
                -RedirectStandardError $BackendErr `
                -PassThru
        } else {
            throw "bridge.exe and Python were not found."
        }
    }

    if (-not (Wait-BackendReady 15)) {
        Write-Fail "Local backend did not become ready."
        if (Test-Path $BackendErr) {
            Write-Host ""
            Write-Host "bridge stderr log:" -ForegroundColor Yellow
            Get-Content -Path $BackendErr
        }
        throw "Check Python and whether ports 8080, 8090, or 14550 are occupied."
    }

    Write-Ok "Local backend is ready."
}

function Stop-BackendProcess {
    if (-not $script:BackendProcess) {
        return
    }
    try {
        if (-not $script:BackendProcess.HasExited) {
            Write-Info ("Stopping local backend pid={0}" -f $script:BackendProcess.Id)
            Stop-Process -Id $script:BackendProcess.Id -Force
        }
    } catch {
        Write-Warn ("Failed to stop backend process: {0}" -f $_.Exception.Message)
    }
}

function Invoke-Api($Method, $Path, $TimeoutSec) {
    return Invoke-RestMethod -Uri "$ApiBase$Path" -Method $Method -TimeoutSec $TimeoutSec
}

function Show-ModuleStatus($StatusBody) {
    $modules = $StatusBody.status.modules
    Write-Host ""
    Write-Host "Module status:" -ForegroundColor White
    Write-Host ("  backend     {0}  {1}" -f $modules.backend.status, $modules.backend.message)
    Write-Host ("  jetson      {0}  {1}" -f $modules.jetson.status, $modules.jetson.message)
    Write-Host ("  arm         {0}  {1}" -f $modules.arm.status, $modules.arm.message)
    Write-Host ("  hand        {0}  {1}" -f $modules.hand.status, $modules.hand.message)
    if ($modules.glove) {
        Write-Host ("  glove       {0}  {1}" -f $modules.glove.status, $modules.glove.message)
    }
    if ($modules.hand_link) {
        Write-Host ("  hand_link   {0}  {1}" -f $modules.hand_link.status, $modules.hand_link.message)
    }
    if ($modules.udp_bridge) {
        Write-Host ("  udp_bridge  {0}  {1}" -f $modules.udp_bridge.status, $modules.udp_bridge.message)
    }
    Write-Host ("  network     {0}  {1}" -f $modules.network.status, $modules.network.message)
    Write-Host ""
}

function Wait-SystemStop {
    $closeLabel = -join ([char[]]@(0x5173, 0x95ed, 0x7cfb, 0x7edf))
    Write-Info ("System is running. Click '" + $closeLabel + "' in the H5 page to stop Jetson nodes and close this window.")
    while ($true) {
        Start-Sleep -Seconds 1
        $status = Get-ApiStatus
        if ($null -eq $status -or -not $status.ok) {
            continue
        }
        $state = $status.status.state
        if ($state -eq "OFFLINE") {
            $script:StopObserved = $true
            Write-Ok "System is offline. One Click Start window will close."
            break
        }
    }
}

function Stop-SystemIfNeeded {
    if (-not $script:StartedSystem -or $script:StopObserved) {
        return
    }
    try {
        Write-Info "Stopping system before One Click Start exits..."
        $stopResult = Invoke-Api Post "/api/system/stop" 25
        if ($stopResult.ok) {
            Write-Ok "System stop command sent."
        } else {
            Write-Warn $stopResult.message
        }
    } catch {
        Write-Warn ("Failed to send system stop command: {0}" -f $_.Exception.Message)
    }
}

try {
    Write-Host ""
    Write-Host "========================================"
    Write-Host " Underwater Arm Console - One Click Start"
    Write-Host "========================================"
    Write-Host ""

    Start-Backend

    Write-Info "Opening H5 console page..."
    Start-Process -FilePath $UiPath

    Write-Info "Starting Jetson nodes and local hand/glove programs through supervisor..."
    $startResult = Invoke-Api Post "/api/system/start" 90
    if ($startResult.status -and $startResult.status.state -ne "OFFLINE") {
        $script:StartedSystem = $true
    }
    Show-ModuleStatus $startResult
    if ($startResult.jetson_check) {
        Write-Host $startResult.jetson_check
    }

    $modules = $startResult.status.modules
    $localReady = (
        $modules.arm.status -eq "ONLINE" -and
        $modules.hand.status -eq "ONLINE" -and
        $modules.glove.status -eq "ONLINE" -and
        $modules.hand_link.status -eq "ONLINE" -and
        $modules.udp_bridge.status -eq "ONLINE"
    )

    if ($startResult.ok -and $localReady) {
        Write-Ok "System is ready. Use the browser page."
    } else {
        Write-Warn "H5 is open, but arm/hand/glove/UDP telemetry link is not fully online. Check the status above."
    }

    Wait-SystemStop
} catch {
    Write-Host ""
    Write-Fail $_.Exception.Message
    Write-Host ""
    Write-Host "Common causes:" -ForegroundColor Yellow
    Write-Host "  1. Windows to Jetson SSH key login is not configured."
    Write-Host "  2. /home/night/robot/start_arm.sh is not deployed on Jetson."
    Write-Host "  3. Ports 8080, 8090, or 14550 are occupied."
    Write-Host "  4. Jetson ROS workspace or launch file names are wrong."
    Write-Host ""
    Write-Host "Manual SSH test:"
    Write-Host '  ssh -o BatchMode=yes -o ConnectTimeout=5 -o UserKnownHostsFile=NUL -o StrictHostKeyChecking=no -o LogLevel=ERROR night@192.168.1.35 "echo SSH_OK"'
    exit 1
} finally {
    Stop-SystemIfNeeded
    Stop-BackendProcess
}
