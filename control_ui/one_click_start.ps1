$ErrorActionPreference = "Stop"

$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $Root
$ProjectRoot = Split-Path -Parent $Root

$utf8 = [System.Text.UTF8Encoding]::new()
[Console]::OutputEncoding = $utf8
$OutputEncoding = $utf8

$ApiBase = "http://127.0.0.1:8090"
$HandTrackerBase = "http://127.0.0.1:8091"
$HandTrackerEnvName = "hand-tracker-arm"
$HandTrackerDir = Join-Path $ProjectRoot "Hand_Tracker"
$HandTrackerLocalEnvDir = Join-Path $HandTrackerDir ".conda\hand-tracker-arm"
$HandTrackerRequirements = Join-Path $HandTrackerDir "requirements.txt"
$HandTrackerScript = Join-Path $HandTrackerDir "hand_arm_control.py"
$HandTrackerConfig = Join-Path $HandTrackerDir "hand_control_config.yaml"
$UiPath = Join-Path $Root "index.html"
$RtspCameraStartScript = Join-Path $Root "start_rtsp_camera.ps1"
$RtspCameraStopScript = Join-Path $Root "stop_rtsp_camera.ps1"
$LogDir = Join-Path $Root "logs"
$BackendOut = Join-Path $LogDir "bridge_stdout.log"
$BackendErr = Join-Path $LogDir "bridge_stderr.log"
$HandTrackerOut = Join-Path $LogDir "hand_tracker_stdout.log"
$HandTrackerErr = Join-Path $LogDir "hand_tracker_stderr.log"
$VrStaticOut = Join-Path $LogDir "vr_static_http_stdout.log"
$VrStaticErr = Join-Path $LogDir "vr_static_http_stderr.log"
$RtspCameraStartOut = Join-Path $LogDir "rtsp_camera_start_stdout.log"
$RtspCameraStartErr = Join-Path $LogDir "rtsp_camera_start_stderr.log"
$RtspCameraPidFile = Join-Path $LogDir "rtsp_camera_pids.json"
$VrStaticPort = 8070
$UiUrl = "http://127.0.0.1:$VrStaticPort/control_ui/index.html"
$script:StartedSystem = $false
$script:StopObserved = $false
$script:BackendProcess = $null
$script:HandTrackerProcess = $null
$script:VrStaticServerProcess = $null
$script:RtspCameraStarted = $false

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

function Get-HandTrackerStatus {
    try {
        return Invoke-RestMethod -Uri "$HandTrackerBase/api/status" -Method Get -TimeoutSec 2
    } catch {
        return $null
    }
}

function Wait-HandTrackerReady($Seconds) {
    $deadline = (Get-Date).AddSeconds($Seconds)
    while ((Get-Date) -lt $deadline) {
        $status = Get-HandTrackerStatus
        if ($null -ne $status -and $status.ok) {
            return $true
        }
        Start-Sleep -Milliseconds 500
    }
    return $false
}

function Get-CondaCommand {
    $cmd = Get-Command conda -ErrorAction SilentlyContinue
    if ($cmd) {
        return $cmd.Source
    }
    $candidates = @(
        "D:\anaconda\Scripts\conda.exe",
        "D:\anaconda\condabin\conda.bat",
        "$env:USERPROFILE\anaconda3\Scripts\conda.exe",
        "$env:USERPROFILE\miniconda3\Scripts\conda.exe"
    )
    foreach ($candidate in $candidates) {
        if (Test-Path $candidate) {
            return $candidate
        }
    }
    return $null
}

function Quote-ProcessArg($Arg) {
    $text = [string]$Arg
    if ($text -notmatch '[\s"]') {
        return $text
    }
    return '"' + ($text -replace '"', '\"') + '"'
}

function Invoke-LoggedProcess($FilePath, $ArgumentString, $WorkingDirectory, $TimeoutSec, $OutFile, $ErrFile, $TimeoutMessage) {
    $psi = [System.Diagnostics.ProcessStartInfo]::new()
    $psi.FileName = $FilePath
    $psi.Arguments = $ArgumentString
    $psi.WorkingDirectory = $WorkingDirectory
    $psi.UseShellExecute = $false
    $psi.CreateNoWindow = $true
    $psi.RedirectStandardOutput = $true
    $psi.RedirectStandardError = $true
    try {
        $psi.StandardOutputEncoding = $utf8
        $psi.StandardErrorEncoding = $utf8
    } catch {}

    $proc = [System.Diagnostics.Process]::new()
    $proc.StartInfo = $psi
    if (-not $proc.Start()) {
        throw "failed to start process: $FilePath"
    }

    $stdoutTask = $proc.StandardOutput.ReadToEndAsync()
    $stderrTask = $proc.StandardError.ReadToEndAsync()
    $timeoutMs = if ($TimeoutSec -and $TimeoutSec -gt 0) {
        [int][Math]::Min(([double]$TimeoutSec * 1000), [int]::MaxValue)
    } else {
        -1
    }
    $exited = if ($timeoutMs -ge 0) {
        $proc.WaitForExit($timeoutMs)
    } else {
        $proc.WaitForExit()
        $true
    }
    if (-not $exited) {
        try { $proc.Kill() } catch {}
        throw $TimeoutMessage
    }
    $proc.WaitForExit()

    $stdout = $stdoutTask.Result
    $stderr = $stderrTask.Result
    [System.IO.File]::WriteAllText($OutFile, $stdout, $utf8)
    [System.IO.File]::WriteAllText($ErrFile, $stderr, $utf8)

    $allText = (($stdout + "`n" + $stderr).Trim())
    if ($allText) {
        Write-Host $allText
    }
    return @{
        Ok = ($proc.ExitCode -eq 0)
        ExitCode = $proc.ExitCode
        Text = $allText
    }
}

function Invoke-Conda($Conda, $Arguments, $TimeoutSec) {
    $argString = (($Arguments | ForEach-Object { Quote-ProcessArg $_ }) -join " ")
    $stamp = "{0:yyyyMMdd_HHmmss}_{1}" -f (Get-Date), ([Guid]::NewGuid().ToString("N").Substring(0, 8))
    $outFile = Join-Path $LogDir "conda_${stamp}_stdout.log"
    $errFile = Join-Path $LogDir "conda_${stamp}_stderr.log"

    $oldNoPlugins = $env:CONDA_NO_PLUGINS
    $oldSolver = $env:CONDA_SOLVER
    $env:CONDA_NO_PLUGINS = "true"
    $env:CONDA_SOLVER = "classic"
    try {
        return Invoke-LoggedProcess $Conda $argString $ProjectRoot $TimeoutSec $outFile $errFile "conda command timed out: $($Arguments -join ' ')"
    } finally {
        if ($null -eq $oldNoPlugins) {
            Remove-Item Env:\CONDA_NO_PLUGINS -ErrorAction SilentlyContinue
        } else {
            $env:CONDA_NO_PLUGINS = $oldNoPlugins
        }
        if ($null -eq $oldSolver) {
            Remove-Item Env:\CONDA_SOLVER -ErrorAction SilentlyContinue
        } else {
            $env:CONDA_SOLVER = $oldSolver
        }
    }
}

function Invoke-EnvPython($PythonExe, $Arguments, $TimeoutSec) {
    $argString = (($Arguments | ForEach-Object { Quote-ProcessArg $_ }) -join " ")
    $stamp = "{0:yyyyMMdd_HHmmss}_{1}" -f (Get-Date), ([Guid]::NewGuid().ToString("N").Substring(0, 8))
    $outFile = Join-Path $LogDir "hand_python_${stamp}_stdout.log"
    $errFile = Join-Path $LogDir "hand_python_${stamp}_stderr.log"
    return Invoke-LoggedProcess $PythonExe $argString $ProjectRoot $TimeoutSec $outFile $errFile "python command timed out: $($Arguments -join ' ')"
}

function Install-HandTrackerPythonDeps($PythonExe) {
    if (-not (Test-Path $HandTrackerRequirements)) {
        throw "Hand tracker requirements file not found: $HandTrackerRequirements"
    }
    Write-Info "Installing/updating hand-recognition Python dependencies..."
    $installed = Invoke-EnvPython $PythonExe @("-m", "pip", "install", "--disable-pip-version-check", "--upgrade", "-r", $HandTrackerRequirements) 1800
    if (-not $installed.Ok) {
        throw "Failed to install hand tracker Python dependencies: $($installed.Text)"
    }
}

function Start-HandTrackerChild($FilePath, $ArgumentList, $WorkingDirectory, $OutFile = $HandTrackerOut, $ErrFile = $HandTrackerErr) {
    return Start-Process -FilePath $FilePath `
        -ArgumentList $ArgumentList `
        -WorkingDirectory $WorkingDirectory `
        -WindowStyle Hidden `
        -RedirectStandardOutput $OutFile `
        -RedirectStandardError $ErrFile `
        -PassThru
}

function Get-CondaEnvPathFromList($Text, $Name) {
    foreach ($line in ($Text -split "`r?`n")) {
        if ($line -match "^\s*$([regex]::Escape($Name))\s+(.+?)\s*$") {
            return $Matches[1].Trim()
        }
    }
    return $null
}

function Ensure-HandTrackerEnv {
    $conda = Get-CondaCommand
    if (-not $conda) {
        throw "conda was not found. Install Anaconda/Miniconda or add conda to PATH."
    }

    Write-Info "Checking hand-recognition conda environment '$HandTrackerEnvName'..."
    $envList = Invoke-Conda $conda @("env", "list") 30
    $envPath = Get-CondaEnvPathFromList $envList.Text $HandTrackerEnvName
    if (-not $envPath) {
        Write-Info "Creating local conda environment for hand recognition..."
        $created = Invoke-Conda $conda @("create", "-y", "--solver", "classic", "-p", $HandTrackerLocalEnvDir, "python=3.10", "pip") 1800
        if (-not $created.Ok) {
            throw "Failed to create hand tracker conda environment: $($created.Text)"
        }
        $envPath = $HandTrackerLocalEnvDir
    }

    $pythonExe = Join-Path $envPath "python.exe"
    if (-not (Test-Path $pythonExe)) {
        throw "Hand tracker python was not found: $pythonExe"
    }

    $verifyCode = "import cv2, mediapipe as mp, yaml, numpy; assert hasattr(mp, 'solutions'), 'mediapipe.solutions missing'; print('HAND_TRACKER_ENV_OK')"
    $verify = Invoke-EnvPython $pythonExe @("-c", $verifyCode) 60
    if (-not $verify.Ok) {
        Write-Warn "Hand tracker dependencies are incomplete. Repairing with pip requirements..."
        Install-HandTrackerPythonDeps $pythonExe
        $verify = Invoke-EnvPython $pythonExe @("-c", $verifyCode) 60
        if (-not $verify.Ok) {
            throw "Hand tracker conda environment verification failed: $($verify.Text)"
        }
    }

    Write-Ok "Hand-recognition environment is ready."
    return $pythonExe
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

function Get-AdbDeviceId($AdbPath) {
    $deviceLines = & $AdbPath devices | Where-Object { $_ -match "^\S+\s+device$" }
    $devices = @($deviceLines | ForEach-Object { ($_ -split "\s+")[0] })

    if ($devices.Count -eq 1) {
        return $devices[0]
    }

    if ($devices.Count -gt 1) {
        Write-Warn "Multiple ADB devices found; using the first one: $($devices[0])"
        return $devices[0]
    }

    return $null
}

function Start-AdbReverseForVr {
    $adb = Find-Adb
    if (-not $adb) {
        Write-Warn "adb.exe was not found. Unity WebView can still start, but headset reverse ports were not configured."
        return
    }

    $deviceId = Get-AdbDeviceId $adb
    if (-not $deviceId) {
        Write-Warn "No authorized ADB device found. Connect/authorize the headset, then run this script again for VR WebView."
        return
    }

    Write-Info "Configuring ADB reverse ports for VR WebView on device $deviceId..."
    foreach ($port in @(8070, 8080, 8090, 8091, 8081, 5005)) {
        & $adb -s $deviceId reverse "tcp:$port" "tcp:$port" | Out-Host
        if ($LASTEXITCODE -eq 0) {
            Write-Ok "adb reverse tcp:$port tcp:$port"
        } else {
            Write-Warn "adb reverse failed on port $port"
        }
    }

    & $adb -s $deviceId reverse --list | Out-Host
}

function Stop-ExistingVrStaticServer {
    try {
        $ownerPids = Get-NetTCPConnection -LocalPort $VrStaticPort -State Listen -ErrorAction SilentlyContinue | Select-Object -ExpandProperty OwningProcess
        foreach ($ownerPid in ($ownerPids | Sort-Object -Unique)) {
            $proc = Get-Process -Id $ownerPid -ErrorAction SilentlyContinue
            if ($proc -and $proc.ProcessName -in @("python", "pythonw")) {
                Write-Info ("Stopping old VR static server pid={0} process={1}" -f $ownerPid, $proc.ProcessName)
                Stop-Process -Id $ownerPid -Force -ErrorAction SilentlyContinue
            } elseif ($proc) {
                Write-Warn ("VR static port {0} is occupied by pid={1} process={2}; not stopping it automatically." -f $VrStaticPort, $ownerPid, $proc.ProcessName)
            }
        }
    } catch {}
    Start-Sleep -Milliseconds 300
}

function Start-VrStaticServer {
    Stop-ExistingVrStaticServer

    $existing = Get-NetTCPConnection -LocalPort $VrStaticPort -State Listen -ErrorAction SilentlyContinue
    if ($existing) {
        Write-Warn "VR static server was not started because port $VrStaticPort is already occupied."
        return
    }

    $python = Get-Command python -ErrorAction SilentlyContinue
    if (-not $python) {
        Write-Warn "Python was not found. Unity WebView static H5 server was not started."
        return
    }

    Remove-Item -LiteralPath $VrStaticOut, $VrStaticErr -Force -ErrorAction SilentlyContinue
    Write-Info "Starting VR static H5 server on http://127.0.0.1:$VrStaticPort/control_ui/index.html ..."
    $script:VrStaticServerProcess = Start-Process -FilePath $python.Source `
        -ArgumentList "-m http.server $VrStaticPort --bind 127.0.0.1" `
        -WorkingDirectory $ProjectRoot `
        -WindowStyle Hidden `
        -RedirectStandardOutput $VrStaticOut `
        -RedirectStandardError $VrStaticErr `
        -PassThru

    Start-Sleep -Milliseconds 800
    $started = Get-NetTCPConnection -LocalPort $VrStaticPort -State Listen -ErrorAction SilentlyContinue
    if ($started) {
        Write-Ok "VR static H5 server is ready."
    } else {
        Write-Warn "VR static H5 server did not begin listening on port $VrStaticPort."
    }
}

function Start-VrWebViewSupport {
    Write-Info "Preparing Unity VR WebView support..."
    Start-AdbReverseForVr
    Start-VrStaticServer
}

function Open-H5Console {
    Write-Info "Opening H5 console page..."
    try {
        Start-Process -FilePath $UiUrl
        Write-Ok "H5 console page requested: $UiUrl"
    } catch {
        Write-Warn ("Failed to open H5 URL: {0}" -f $_.Exception.Message)
        Write-Warn "Falling back to local index.html."
        Start-Process -FilePath $UiPath
    }
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

function Stop-ExistingHandTracker {
    try {
        Get-CimInstance Win32_Process -Filter "name = 'python.exe' or name = 'pythonw.exe' or name = 'conda.exe' or name = 'cmd.exe'" |
            Where-Object { $_.CommandLine -like "*hand_arm_control.py*" } |
            ForEach-Object {
                Write-Info ("Stopping old hand tracker pid={0}" -f $_.ProcessId)
                Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue
            }
    } catch {}

    try {
        $ownerPids = Get-NetTCPConnection -LocalPort 8091 -State Listen -ErrorAction SilentlyContinue | Select-Object -ExpandProperty OwningProcess
        foreach ($ownerPid in ($ownerPids | Sort-Object -Unique)) {
            $proc = Get-Process -Id $ownerPid -ErrorAction SilentlyContinue
            if ($proc -and $proc.ProcessName -in @("python", "pythonw", "conda", "cmd")) {
                Write-Info ("Stopping process on hand tracker port pid={0} process={1}" -f $ownerPid, $proc.ProcessName)
                Stop-Process -Id $ownerPid -Force -ErrorAction SilentlyContinue
            }
        }
    } catch {}

    Start-Sleep -Milliseconds 500
}

function Start-HandTracker {
    Stop-ExistingHandTracker

    Write-Info "Starting VR wrist coordinate service..."
    Remove-Item -LiteralPath $HandTrackerOut, $HandTrackerErr -Force -ErrorAction SilentlyContinue

    $pythonExe = $null
    try {
        $pythonExe = Ensure-HandTrackerEnv
    } catch {
        Write-Warn ("Hand tracker conda environment is unavailable: {0}" -f $_.Exception.Message)
        $python = Get-Command python -ErrorAction SilentlyContinue
        if ($python) {
            Write-Warn "Falling back to PATH python for VR wrist coordinate service."
            $pythonExe = $python.Source
        }
    }
    if (-not $pythonExe) {
        Write-Warn "Python was not found. VR wrist coordinate service was not started."
        return $false
    }
    $script:HandTrackerProcess = Start-HandTrackerChild $pythonExe "-u `"$HandTrackerScript`" --config `"$HandTrackerConfig`" --host 0.0.0.0 --port 8091 --input-source vr --vr-host 127.0.0.1 --vr-port 5005" $ProjectRoot

    if (-not (Wait-HandTrackerReady 25)) {
        Write-Warn "VR wrist coordinate service did not become ready. The H5 page can still use keyboard/gamepad."
        $status = Get-HandTrackerStatus
        if ($status -and $status.message) {
            Write-Warn ("VR wrist status: {0}" -f $status.message)
        }
        if (Test-Path $HandTrackerOut) {
            Write-Host ""
            Write-Host "VR wrist stdout log:" -ForegroundColor Yellow
            Get-Content -Path $HandTrackerOut
        }
        if (Test-Path $HandTrackerErr) {
            Write-Host ""
            Write-Host "VR wrist stderr log:" -ForegroundColor Yellow
            Get-Content -Path $HandTrackerErr
        }
        return $false
    }

    Write-Ok "VR wrist coordinate service is ready."
    return $true
}

function Start-RtspCameraStream {
    if (-not (Test-Path $RtspCameraStartScript)) {
        Write-Warn "RTSP camera startup script was not found: $RtspCameraStartScript"
        return $false
    }

    Write-Info "Starting RTSP USB camera stream..."
    Remove-Item -LiteralPath $RtspCameraStartOut, $RtspCameraStartErr -Force -ErrorAction SilentlyContinue
    $powershellExe = Join-Path $PSHOME "powershell.exe"
    if (-not (Test-Path $powershellExe)) {
        $powershellCmd = Get-Command powershell.exe -ErrorAction SilentlyContinue
        if ($powershellCmd) {
            $powershellExe = $powershellCmd.Source
        }
    }
    if (-not (Test-Path $powershellExe)) {
        Write-Warn "powershell.exe was not found. RTSP USB camera stream was not started."
        return $false
    }
    $args = @("-NoProfile", "-ExecutionPolicy", "Bypass", "-File", $RtspCameraStartScript)
    $argString = (($args | ForEach-Object { Quote-ProcessArg $_ }) -join " ")
    $startTime = Get-Date
    $proc = Start-Process -FilePath $powershellExe `
        -ArgumentList $argString `
        -WorkingDirectory $Root `
        -WindowStyle Hidden `
        -RedirectStandardOutput $RtspCameraStartOut `
        -RedirectStandardError $RtspCameraStartErr `
        -PassThru

    $ready = $false
    $deadline = (Get-Date).AddSeconds(45)
    while ((Get-Date) -lt $deadline) {
        if (Test-Path $RtspCameraPidFile) {
            try {
                $pidFile = Get-Item -LiteralPath $RtspCameraPidFile -ErrorAction Stop
                if ($pidFile.LastWriteTime -ge $startTime.AddSeconds(-2)) {
                    $pids = Get-Content -Path $RtspCameraPidFile -Raw | ConvertFrom-Json
                    $ffmpegProc = if ($pids.ffmpegPid) { Get-Process -Id $pids.ffmpegPid -ErrorAction SilentlyContinue } else { $null }
                    $mediaMtxProc = if ($pids.mediaMtxPid) { Get-Process -Id $pids.mediaMtxPid -ErrorAction SilentlyContinue } else { $null }
                    if ($ffmpegProc -and $mediaMtxProc) {
                        $ready = $true
                        break
                    }
                }
            } catch {}
        }
        if ($proc.HasExited -and -not $ready) {
            break
        }
        Start-Sleep -Milliseconds 300
    }

    Start-Sleep -Milliseconds 300
    if (Test-Path $RtspCameraStartOut) {
        Get-Content -Path $RtspCameraStartOut | Out-Host
    }
    if (Test-Path $RtspCameraStartErr) {
        Get-Content -Path $RtspCameraStartErr | Out-Host
    }

    if ($ready) {
        if (-not $proc.HasExited) {
            Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue
        }
        $script:RtspCameraStarted = $true
        return $true
    }

    if (-not $proc.HasExited) {
        Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue
    }
    Write-Warn "RTSP USB camera stream was not started. Unity LibVLC panel will stay offline until MediaMTX/FFmpeg are available."
    return $false
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

function Stop-HandTrackerProcess {
    if (-not $script:HandTrackerProcess) {
        return
    }
    try {
        if (-not $script:HandTrackerProcess.HasExited) {
            Write-Info ("Stopping hand-recognition service pid={0}" -f $script:HandTrackerProcess.Id)
            Stop-Process -Id $script:HandTrackerProcess.Id -Force
        }
    } catch {
        Write-Warn ("Failed to stop hand-recognition service: {0}" -f $_.Exception.Message)
    }
}

function Stop-RtspCameraStream {
    if (-not $script:RtspCameraStarted -or -not (Test-Path $RtspCameraStopScript)) {
        return
    }

    try {
        & powershell -NoProfile -ExecutionPolicy Bypass -File $RtspCameraStopScript | Out-Host
    } catch {
        Write-Warn ("Failed to stop RTSP USB camera stream: {0}" -f $_.Exception.Message)
    }
}

function Stop-VrStaticServerProcess {
    if (-not $script:VrStaticServerProcess) {
        return
    }
    try {
        if (-not $script:VrStaticServerProcess.HasExited) {
            Write-Info ("Stopping VR static H5 server pid={0}" -f $script:VrStaticServerProcess.Id)
            Stop-Process -Id $script:VrStaticServerProcess.Id -Force
        }
    } catch {
        Write-Warn ("Failed to stop VR static H5 server: {0}" -f $_.Exception.Message)
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

    Start-VrWebViewSupport
    Start-Backend
    Open-H5Console
    $wristServiceReady = Start-HandTracker
    $rtspCameraReady = Start-RtspCameraStream

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
    if (-not $wristServiceReady) {
        Write-Warn "VR wrist coordinate input is unavailable until the local wrist service starts."
    }
    if (-not $rtspCameraReady) {
        Write-Warn "RTSP USB camera stream is unavailable until FFmpeg and MediaMTX are configured."
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
    Stop-RtspCameraStream
    Stop-HandTrackerProcess
    Stop-BackendProcess
    Stop-VrStaticServerProcess
}
