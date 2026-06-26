param(
    [string]$LeftCameraName = "",
    [string]$RightCameraName = "",
    [string]$StreamHost = "",
    [int]$RtspPort = 0,
    [string]$StreamPath = "",
    [string]$VideoSize = "",
    [int]$Framerate = 0,
    [string]$LeftInputCodec = "",
    [string]$RightInputCodec = "",
    [string]$LeftVideoPinName = "",
    [string]$RightVideoPinName = "",
    [string]$Layout = "",
    [string]$CaptureBackend = "",
    [int]$LeftCameraIndex = -1,
    [int]$RightCameraIndex = -1,
    [string]$InputFourcc = "",
    [int]$StreamCrf = 0,
    [string]$PythonExe = ""
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectRoot = Split-Path -Parent $Root
$ConfigPath = Join-Path $Root "config.yaml"
$LogDir = Join-Path $Root "logs"
$RuntimeMediaMtxConfig = Join-Path $LogDir "mediamtx_rtsp_camera_runtime.yml"
$PidFile = Join-Path $LogDir "rtsp_camera_pids.json"
$MediaMtxOut = Join-Path $LogDir "mediamtx_stdout.log"
$MediaMtxErr = Join-Path $LogDir "mediamtx_stderr.log"
$FfmpegOut = Join-Path $LogDir "ffmpeg_rtsp_stdout.log"
$FfmpegErr = Join-Path $LogDir "ffmpeg_rtsp_stderr.log"
$LatestFrameScript = Join-Path $Root "latest_frame_dual_stream.py"
$LatestFrameOut = Join-Path $LogDir "latest_frame_stream_stdout.log"
$LatestFrameErr = Join-Path $LogDir "latest_frame_stream_stderr.log"
$LatestFrameStatus = Join-Path $LogDir "latest_frame_stream_status.json"
$LatestFrameFfmpegLog = Join-Path $LogDir "latest_frame_ffmpeg.log"
New-Item -ItemType Directory -Force -Path $LogDir | Out-Null

function Write-Info($Text) { Write-Host "[INFO] $Text" -ForegroundColor Cyan }
function Write-Ok($Text) { Write-Host "[OK] $Text" -ForegroundColor Green }
function Write-Warn($Text) { Write-Host "[WARN] $Text" -ForegroundColor Yellow }

function Get-ConfigSection($Path, $SectionName) {
    $result = @{}
    if (-not (Test-Path $Path)) { return $result }
    $inside = $false
    foreach ($raw in Get-Content -Path $Path) {
        $line = ($raw -split "#", 2)[0].TrimEnd()
        if (-not $line.Trim()) { continue }
        if ($line -notmatch "^\s") {
            $inside = ($line.Trim() -eq "$SectionName`:")
            continue
        }
        if ($inside -and $line -match "^\s+([^:]+):\s*(.*)$") {
            $key = $Matches[1].Trim()
            $value = $Matches[2].Trim()
            if ($value.StartsWith('"') -and $value.EndsWith('"')) {
                $value = $value.Substring(1, $value.Length - 2)
            }
            $result[$key] = $value
        }
    }
    return $result
}

function Get-ConfigValue($Section, $Name, $Fallback) {
    if ($Section.ContainsKey($Name) -and $null -ne $Section[$Name] -and "$($Section[$Name])" -ne "") {
        return $Section[$Name]
    }
    return $Fallback
}

function Get-ConfigValueAllowEmpty($Section, $Name, $Fallback) {
    if ($Section.ContainsKey($Name) -and $null -ne $Section[$Name]) {
        return $Section[$Name]
    }
    return $Fallback
}

function Find-Executable($Name, [string[]]$ExtraCandidates) {
    $cmd = Get-Command $Name -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }
    foreach ($candidate in $ExtraCandidates) {
        if ($candidate -and (Test-Path $candidate)) { return $candidate }
    }
    return $null
}

function Test-PythonCv2($PythonPath) {
    if (-not $PythonPath -or -not (Test-Path $PythonPath)) { return $false }
    try {
        & $PythonPath -c "import cv2, numpy" *> $null
        return ($LASTEXITCODE -eq 0)
    } catch {
        return $false
    }
}

function Find-PythonWithCv2($Preferred) {
    $pathPython = Get-Command "python.exe" -ErrorAction SilentlyContinue
    $candidates = @(
        $Preferred,
        "D:\AppData\envs\lerobot\python.exe",
        "D:\AppData\envs\hand-tracker-arm\python.exe",
        "D:\AppData\envs\wa100\python.exe",
        "D:\anaconda\python.exe",
        $(if ($pathPython) { $pathPython.Source } else { $null })
    ) | Where-Object { $_ } | Select-Object -Unique

    foreach ($candidate in $candidates) {
        if (Test-PythonCv2 $candidate) {
            return $candidate
        }
    }
    return $null
}

function Stop-ProcessIfPresent($Id) {
    if (-not $Id) { return }
    $proc = Get-Process -Id $Id -ErrorAction SilentlyContinue
    if ($proc) {
        Stop-Process -Id $Id -Force -ErrorAction SilentlyContinue
    }
}

function Quote-Arg($Arg) {
    $text = [string]$Arg
    if ($text -notmatch '[\s"]') { return $text }
    return '"' + ($text -replace '"', '\"') + '"'
}

function Join-Args([string[]]$ArgumentItems) {
    return (($ArgumentItems | ForEach-Object { Quote-Arg $_ }) -join " ")
}

function Test-TcpPort($HostName, $Port) {
    $client = [System.Net.Sockets.TcpClient]::new()
    try {
        $async = $client.BeginConnect($HostName, $Port, $null, $null)
        if (-not $async.AsyncWaitHandle.WaitOne(400)) { return $false }
        $client.EndConnect($async)
        return $true
    } catch {
        return $false
    } finally {
        $client.Close()
    }
}

function Wait-TcpPort($HostName, $Port, $Seconds) {
    $deadline = (Get-Date).AddSeconds($Seconds)
    while ((Get-Date) -lt $deadline) {
        if (Test-TcpPort $HostName $Port) { return $true }
        Start-Sleep -Milliseconds 300
    }
    return $false
}

function Stop-OldRtspProcesses {
    $stopScript = Join-Path $Root "stop_rtsp_camera.ps1"
    if (Test-Path $stopScript) {
        try {
            & powershell -NoProfile -ExecutionPolicy Bypass -File $stopScript | Out-Host
            Start-Sleep -Milliseconds 300
            return
        } catch {
            Write-Warn ("Failed to run RTSP stop script: {0}" -f $_.Exception.Message)
        }
    }
    if (-not (Test-Path $PidFile)) { return }
    try {
        $pids = Get-Content -Path $PidFile -Raw | ConvertFrom-Json
        foreach ($id in @($pids.ffmpegPid, $pids.mediaMtxPid)) {
            if (-not $id) { continue }
            $proc = Get-Process -Id $id -ErrorAction SilentlyContinue
            if ($proc) {
                Write-Info ("Stopping old RTSP helper pid={0} process={1}" -f $id, $proc.ProcessName)
                Stop-Process -Id $id -Force -ErrorAction SilentlyContinue
            }
        }
    } catch {
        Write-Warn ("Failed to read old RTSP pid file: {0}" -f $_.Exception.Message)
    }
    Remove-Item -LiteralPath $PidFile -Force -ErrorAction SilentlyContinue
    Start-Sleep -Milliseconds 300
}

function Add-DshowInput([System.Collections.Generic.List[string]]$ArgumentItems, [string]$CameraName, [string]$Codec, [string]$PinName, [string]$Size, [int]$Fps) {
    $ArgumentItems.Add("-f")
    $ArgumentItems.Add("dshow")
    $ArgumentItems.Add("-rtbufsize")
    $ArgumentItems.Add("256M")
    $ArgumentItems.Add("-framerate")
    $ArgumentItems.Add("$Fps")
    $ArgumentItems.Add("-video_size")
    $ArgumentItems.Add("$Size")
    if ($Codec) {
        $ArgumentItems.Add("-vcodec")
        $ArgumentItems.Add("$Codec")
    }
    if ($PinName) {
        $ArgumentItems.Add("-video_pin_name")
        $ArgumentItems.Add("$PinName")
    }
    $ArgumentItems.Add("-i")
    $ArgumentItems.Add("video=$CameraName")
}

function Split-VideoSize($Size) {
    if ($Size -notmatch '^(\d+)x(\d+)$') {
        throw "VideoSize must look like 640x480, got: $Size"
    }
    return @{ Width = [int]$Matches[1]; Height = [int]$Matches[2] }
}

$dualCfg = Get-ConfigSection $ConfigPath "rtsp_dual_camera"
$enabled = (Get-ConfigValue $dualCfg "enabled" "true").ToString().ToLowerInvariant()
if ($enabled -in @("0", "false", "no", "off")) {
    Write-Warn "Dual RTSP camera stream is disabled in config.yaml."
    exit 0
}

if (-not $LeftCameraName) { $LeftCameraName = Get-ConfigValue $dualCfg "left_video_device" "RGB Camera" }
if (-not $RightCameraName) { $RightCameraName = Get-ConfigValue $dualCfg "right_video_device" "USB Camera" }
if (-not $StreamHost) { $StreamHost = Get-ConfigValue $dualCfg "stream_host" "127.0.0.1" }
if (-not $RtspPort) { $RtspPort = [int](Get-ConfigValue $dualCfg "rtsp_port" "8554") }
if (-not $StreamPath) { $StreamPath = Get-ConfigValue $dualCfg "stream_path" "usb_camera" }
if (-not $VideoSize) { $VideoSize = Get-ConfigValue $dualCfg "video_size" "640x960" }
if (-not $Framerate) { $Framerate = [int](Get-ConfigValue $dualCfg "framerate" "15") }
if (-not $LeftInputCodec) { $LeftInputCodec = Get-ConfigValue $dualCfg "left_input_codec" "mjpeg" }
if (-not $RightInputCodec) { $RightInputCodec = Get-ConfigValue $dualCfg "right_input_codec" "h264" }
if (-not $LeftVideoPinName) { $LeftVideoPinName = Get-ConfigValueAllowEmpty $dualCfg "left_video_pin_name" "" }
if (-not $RightVideoPinName) { $RightVideoPinName = Get-ConfigValueAllowEmpty $dualCfg "right_video_pin_name" "1" }
if (-not $Layout) { $Layout = Get-ConfigValue $dualCfg "layout" "hstack" }
if (-not $CaptureBackend) { $CaptureBackend = Get-ConfigValue $dualCfg "capture_backend" "latest_frame" }
if ($LeftCameraIndex -lt 0) { $LeftCameraIndex = [int](Get-ConfigValue $dualCfg "left_camera_index" "0") }
if ($RightCameraIndex -lt 0) { $RightCameraIndex = [int](Get-ConfigValue $dualCfg "right_camera_index" "1") }
if (-not $InputFourcc) { $InputFourcc = Get-ConfigValue $dualCfg "input_fourcc" "MJPG" }
if ($StreamCrf -le 0) { $StreamCrf = [int](Get-ConfigValue $dualCfg "stream_crf" "26") }
if (-not $PythonExe) { $PythonExe = Get-ConfigValue $dualCfg "python_exe" "" }

$size = Split-VideoSize $VideoSize
$filterFps = [Math]::Max($Framerate, 1)
$leftChain = "[0:v]fps=fps=$filterFps,scale=$($size.Width):$($size.Height),setsar=1,setpts=N/($filterFps*TB)[left]"
$rightChain = "[1:v]fps=fps=$filterFps,scale=$($size.Width):$($size.Height),setsar=1,setpts=N/($filterFps*TB)[right]"
if ($Layout -eq "vstack") {
    $stackChain = "[left][right]vstack=inputs=2,format=yuv420p[out]"
} else {
    $stackChain = "[left][right]hstack=inputs=2,format=yuv420p[out]"
}
$filterComplex = "$leftChain;$rightChain;$stackChain"

$ffmpeg = Find-Executable "ffmpeg.exe" @(
    "C:\Users\13734\anaconda3\Library\bin\ffmpeg.exe",
    (Join-Path $Root "tools\ffmpeg\bin\ffmpeg.exe"),
    (Join-Path $ProjectRoot "tools\ffmpeg\bin\ffmpeg.exe")
)
$mediaMtx = Find-Executable "mediamtx.exe" @(
    (Join-Path $Root "tools\mediamtx\mediamtx.exe"),
    (Join-Path $ProjectRoot "tools\mediamtx\mediamtx.exe"),
    (Join-Path $Root "mediamtx.exe"),
    (Join-Path $ProjectRoot "mediamtx.exe")
)

if (-not $ffmpeg) {
    Write-Warn "ffmpeg.exe was not found. Install FFmpeg or put it under control_ui\tools\ffmpeg\bin."
    exit 2
}
if (-not $mediaMtx) {
    Write-Warn "mediamtx.exe was not found. Put mediamtx.exe in PATH or control_ui\tools\mediamtx\mediamtx.exe."
    exit 2
}

Stop-OldRtspProcesses

$rtspPublishUrl = "rtsp://127.0.0.1:$RtspPort/$StreamPath"
$rtspViewUrl = "rtsp://$StreamHost`:$RtspPort/$StreamPath"

Remove-Item -LiteralPath $MediaMtxOut, $MediaMtxErr, $FfmpegOut, $FfmpegErr, $LatestFrameOut, $LatestFrameErr, $LatestFrameStatus, $LatestFrameFfmpegLog -Force -ErrorAction SilentlyContinue
$mediaMtxConfigText = @"
logLevel: info
rtsp: true
rtspAddress: :$RtspPort
rtspTransports: [tcp]
rtmp: false
hls: false
webrtc: false
srt: false
moq: false

paths:
  ${StreamPath}:
    source: publisher
  all_others:
    source: publisher
"@
[System.IO.File]::WriteAllText(
    $RuntimeMediaMtxConfig,
    $mediaMtxConfigText,
    [System.Text.UTF8Encoding]::new($false)
)

Write-Info "Starting MediaMTX on RTSP port $RtspPort..."
$mediaMtxProc = Start-Process -FilePath $mediaMtx `
    -ArgumentList (Quote-Arg $RuntimeMediaMtxConfig) `
    -WorkingDirectory $Root `
    -WindowStyle Hidden `
    -RedirectStandardOutput $MediaMtxOut `
    -RedirectStandardError $MediaMtxErr `
    -PassThru

if (-not (Wait-TcpPort "127.0.0.1" $RtspPort 8)) {
    Write-Warn "MediaMTX did not begin listening on port $RtspPort."
    if (Test-Path $MediaMtxErr) { Get-Content -Path $MediaMtxErr }
    Stop-Process -Id $mediaMtxProc.Id -Force -ErrorAction SilentlyContinue
    exit 2
}

$backend = $CaptureBackend.ToString().ToLowerInvariant()
if ($backend -in @("latest_frame", "latest", "opencv")) {
    if (-not (Test-Path $LatestFrameScript)) {
        Write-Warn "Latest-frame camera script was not found: $LatestFrameScript"
        Stop-Process -Id $mediaMtxProc.Id -Force -ErrorAction SilentlyContinue
        exit 2
    }

    $python = Find-PythonWithCv2 $PythonExe
    if (-not $python) {
        Write-Warn "No Python with cv2 was found. Install opencv-python or set rtsp_dual_camera.python_exe in config.yaml."
        Stop-Process -Id $mediaMtxProc.Id -Force -ErrorAction SilentlyContinue
        exit 2
    }

    Write-Info "Starting latest-frame dual capture with $python"
    Write-Info "OpenCV camera indexes: left=$LeftCameraIndex ($LeftCameraName), right=$RightCameraIndex ($RightCameraName)"

    $captureArgs = [System.Collections.Generic.List[string]]::new()
    $captureArgs.Add("-u")
    $captureArgs.Add($LatestFrameScript)
    $captureArgs.Add("--left-index")
    $captureArgs.Add("$LeftCameraIndex")
    $captureArgs.Add("--right-index")
    $captureArgs.Add("$RightCameraIndex")
    $captureArgs.Add("--left-name")
    $captureArgs.Add($LeftCameraName)
    $captureArgs.Add("--right-name")
    $captureArgs.Add($RightCameraName)
    $captureArgs.Add("--width")
    $captureArgs.Add("$($size.Width)")
    $captureArgs.Add("--height")
    $captureArgs.Add("$($size.Height)")
    $captureArgs.Add("--fps")
    $captureArgs.Add("$Framerate")
    $captureArgs.Add("--layout")
    $captureArgs.Add($Layout)
    $captureArgs.Add("--fourcc")
    $captureArgs.Add($InputFourcc)
    $captureArgs.Add("--ffmpeg")
    $captureArgs.Add($ffmpeg)
    $captureArgs.Add("--output")
    $captureArgs.Add($rtspPublishUrl)
    $captureArgs.Add("--status-json")
    $captureArgs.Add($LatestFrameStatus)
    $captureArgs.Add("--ffmpeg-log")
    $captureArgs.Add($LatestFrameFfmpegLog)
    $captureArgs.Add("--crf")
    $captureArgs.Add("$StreamCrf")

    $captureCommandLine = Join-Args @($captureArgs.ToArray())
    $captureCommandLine | Set-Content -Path (Join-Path $LogDir "latest_frame_stream_command.txt") -Encoding UTF8

    $captureProc = Start-Process -FilePath $python `
        -ArgumentList $captureCommandLine `
        -WorkingDirectory $Root `
        -WindowStyle Hidden `
        -RedirectStandardOutput $LatestFrameOut `
        -RedirectStandardError $LatestFrameErr `
        -PassThru

    $ready = $false
    $status = $null
    $deadline = (Get-Date).AddSeconds(14)
    while ((Get-Date) -lt $deadline) {
        if ($captureProc.HasExited) {
            break
        }
        if (Test-Path $LatestFrameStatus) {
            try {
                $status = Get-Content -Path $LatestFrameStatus -Raw | ConvertFrom-Json
                $encoderProc = if ($status.ffmpegPid) { Get-Process -Id $status.ffmpegPid -ErrorAction SilentlyContinue } else { $null }
                if ($encoderProc -and $status.writtenFrames -and [int]$status.writtenFrames -gt 0) {
                    $ready = $true
                    break
                }
            } catch {}
        }
        Start-Sleep -Milliseconds 300
    }

    if (-not $ready) {
        Write-Warn "Latest-frame dual capture did not become ready."
        if (Test-Path $LatestFrameOut) { Get-Content -Path $LatestFrameOut | Out-Host }
        if (Test-Path $LatestFrameErr) { Get-Content -Path $LatestFrameErr | Out-Host }
        if (Test-Path $LatestFrameFfmpegLog) { Get-Content -Path $LatestFrameFfmpegLog | Out-Host }
        if ($status -and $status.ffmpegPid) { Stop-ProcessIfPresent $status.ffmpegPid }
        Stop-ProcessIfPresent $captureProc.Id
        Stop-ProcessIfPresent $mediaMtxProc.Id
        exit 2
    }

    @{
        mode = "latest_frame_dual"
        mediaMtxPid = $mediaMtxProc.Id
        ffmpegPid = $captureProc.Id
        capturePid = $captureProc.Id
        encoderPid = $status.ffmpegPid
        rtspUrl = $rtspViewUrl
        leftCameraName = $LeftCameraName
        rightCameraName = $RightCameraName
        leftCameraIndex = $LeftCameraIndex
        rightCameraIndex = $RightCameraIndex
        videoSize = $VideoSize
        outputSize = $status.size
        framerate = $Framerate
        layout = $Layout
        backend = $CaptureBackend
        startedAt = (Get-Date).ToString("o")
    } | ConvertTo-Json | Set-Content -Path $PidFile -Encoding UTF8

    Write-Ok "Latest-frame dual RTSP camera stream is ready: $rtspViewUrl"
    Write-Ok ("Output size: {0}, capture fps: {1}, frame age: {2:N0}/{3:N0} ms" -f $status.size, $status.writeFps, $status.leftAgeMs, $status.rightAgeMs)
    exit 0
}

Write-Info "Starting dual FFmpeg capture: '$LeftCameraName' + '$RightCameraName' -> $rtspPublishUrl"
$ffmpegArgs = [System.Collections.Generic.List[string]]::new()
$ffmpegArgs.Add("-hide_banner")
$ffmpegArgs.Add("-loglevel")
$ffmpegArgs.Add("warning")
$ffmpegArgs.Add("-fflags")
$ffmpegArgs.Add("+genpts+nobuffer")
Add-DshowInput $ffmpegArgs $LeftCameraName $LeftInputCodec $LeftVideoPinName $VideoSize $Framerate
Add-DshowInput $ffmpegArgs $RightCameraName $RightInputCodec $RightVideoPinName $VideoSize $Framerate
$ffmpegArgs.Add("-filter_complex")
$ffmpegArgs.Add($filterComplex)
$ffmpegArgs.Add("-map")
$ffmpegArgs.Add("[out]")
$ffmpegArgs.Add("-an")
$ffmpegArgs.Add("-c:v")
$ffmpegArgs.Add("libx264")
$ffmpegArgs.Add("-preset")
$ffmpegArgs.Add("ultrafast")
$ffmpegArgs.Add("-tune")
$ffmpegArgs.Add("zerolatency")
$ffmpegArgs.Add("-pix_fmt")
$ffmpegArgs.Add("yuv420p")
$ffmpegArgs.Add("-r")
$ffmpegArgs.Add("$Framerate")
$ffmpegArgs.Add("-g")
$ffmpegArgs.Add("$Framerate")
$ffmpegArgs.Add("-bf")
$ffmpegArgs.Add("0")
$ffmpegArgs.Add("-muxdelay")
$ffmpegArgs.Add("0")
$ffmpegArgs.Add("-muxpreload")
$ffmpegArgs.Add("0")
$ffmpegArgs.Add("-f")
$ffmpegArgs.Add("rtsp")
$ffmpegArgs.Add("-rtsp_transport")
$ffmpegArgs.Add("tcp")
$ffmpegArgs.Add($rtspPublishUrl)

$ffmpegCommandLine = Join-Args @($ffmpegArgs.ToArray())
$ffmpegCommandLine | Set-Content -Path (Join-Path $LogDir "ffmpeg_rtsp_command.txt") -Encoding UTF8

$ffmpegProc = Start-Process -FilePath $ffmpeg `
    -ArgumentList $ffmpegCommandLine `
    -WorkingDirectory $Root `
    -WindowStyle Hidden `
    -RedirectStandardOutput $FfmpegOut `
    -RedirectStandardError $FfmpegErr `
    -PassThru

Start-Sleep -Seconds 3
if ($ffmpegProc.HasExited) {
    Write-Warn "FFmpeg exited immediately. Check $FfmpegErr."
    if (Test-Path $FfmpegErr) { Get-Content -Path $FfmpegErr }
    Stop-Process -Id $mediaMtxProc.Id -Force -ErrorAction SilentlyContinue
    exit 2
}

@{
    mode = "dual"
    mediaMtxPid = $mediaMtxProc.Id
    ffmpegPid = $ffmpegProc.Id
    rtspUrl = $rtspViewUrl
    leftCameraName = $LeftCameraName
    rightCameraName = $RightCameraName
    videoSize = $VideoSize
    framerate = $Framerate
    layout = $Layout
    startedAt = (Get-Date).ToString("o")
} | ConvertTo-Json | Set-Content -Path $PidFile -Encoding UTF8

Write-Ok "Dual RTSP camera stream is ready: $rtspViewUrl"
exit 0
