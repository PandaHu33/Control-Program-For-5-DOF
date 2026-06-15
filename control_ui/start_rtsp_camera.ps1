param(
    [string]$CameraName = "",
    [string]$StreamHost = "",
    [int]$RtspPort = 0,
    [string]$StreamPath = "",
    [string]$VideoSize = "",
    [int]$Framerate = 0
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

function Find-Executable($Name, [string[]]$ExtraCandidates) {
    $cmd = Get-Command $Name -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }
    foreach ($candidate in $ExtraCandidates) {
        if ($candidate -and (Test-Path $candidate)) { return $candidate }
    }
    return $null
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

function Get-DirectShowVideoDevices($FfmpegPath) {
    $previousErrorActionPreference = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    try {
        $output = & $FfmpegPath -hide_banner -list_devices true -f dshow -i dummy 2>&1 | Out-String
    } finally {
        $ErrorActionPreference = $previousErrorActionPreference
    }

    $devices = New-Object System.Collections.Generic.List[string]
    foreach ($line in ($output -split "`r?`n")) {
        if ($line -match '"([^"]+)"\s+\(video\)') {
            $devices.Add($Matches[1])
        }
    }
    return @($devices)
}

function Choose-CameraDevice($FfmpegPath, $PreferredName) {
    if ($PreferredName) { return $PreferredName }
    $devices = Get-DirectShowVideoDevices $FfmpegPath
    if (-not $devices -or $devices.Count -eq 0) {
        throw "No DirectShow video devices were found by FFmpeg."
    }
    $filtered = @($devices | Where-Object { $_ -notmatch '(?i)virtual|obs|screen|unity|tetherscript' })
    if ($filtered.Count -gt 0) { return $filtered[0] }
    return $devices[0]
}

function Stop-OldRtspProcesses {
    if (-not (Test-Path $PidFile)) { return }
    try {
        $pids = Get-Content -Path $PidFile -Raw | ConvertFrom-Json
        foreach ($id in @($pids.ffmpegPid, $pids.mediaMtxPid)) {
            if ($id) {
                $proc = Get-Process -Id $id -ErrorAction SilentlyContinue
                if ($proc) {
                    Write-Info ("Stopping old RTSP helper pid={0} process={1}" -f $id, $proc.ProcessName)
                    Stop-Process -Id $id -Force -ErrorAction SilentlyContinue
                }
            }
        }
    } catch {
        Write-Warn ("Failed to read old RTSP pid file: {0}" -f $_.Exception.Message)
    }
    Remove-Item -LiteralPath $PidFile -Force -ErrorAction SilentlyContinue
    Start-Sleep -Milliseconds 300
}

$cfg = Get-ConfigSection $ConfigPath "rtsp_camera"
$enabled = (Get-ConfigValue $cfg "enabled" "true").ToString().ToLowerInvariant()
if ($enabled -in @("0", "false", "no", "off")) {
    Write-Warn "RTSP camera stream is disabled in config.yaml."
    exit 0
}

if (-not $CameraName) { $CameraName = Get-ConfigValue $cfg "video_device" "" }
if (-not $StreamHost) { $StreamHost = Get-ConfigValue $cfg "stream_host" "127.0.0.1" }
if (-not $RtspPort) { $RtspPort = [int](Get-ConfigValue $cfg "rtsp_port" "8554") }
if (-not $StreamPath) { $StreamPath = Get-ConfigValue $cfg "stream_path" "usb_camera" }
if (-not $VideoSize) { $VideoSize = Get-ConfigValue $cfg "video_size" "1280x720" }
if (-not $Framerate) { $Framerate = [int](Get-ConfigValue $cfg "framerate" "30") }

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

$CameraName = Choose-CameraDevice $ffmpeg $CameraName
$rtspPublishUrl = "rtsp://127.0.0.1:$RtspPort/$StreamPath"
$rtspViewUrl = "rtsp://$StreamHost`:$RtspPort/$StreamPath"

Remove-Item -LiteralPath $MediaMtxOut, $MediaMtxErr, $FfmpegOut, $FfmpegErr -Force -ErrorAction SilentlyContinue
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
    exit 2
}

Write-Info "Starting FFmpeg DirectShow capture: $CameraName -> $rtspPublishUrl"
$ffmpegArgs = @(
    "-hide_banner",
    "-loglevel", "warning",
    "-f", "dshow",
    "-rtbufsize", "256M",
    "-framerate", "$Framerate",
    "-video_size", "$VideoSize",
    "-i", "video=$CameraName",
    "-an",
    "-c:v", "libx264",
    "-preset", "ultrafast",
    "-tune", "zerolatency",
    "-pix_fmt", "yuv420p",
    "-g", "$Framerate",
    "-bf", "0",
    "-f", "rtsp",
    "-rtsp_transport", "tcp",
    $rtspPublishUrl
)
$ffmpegProc = Start-Process -FilePath $ffmpeg `
    -ArgumentList (Join-Args $ffmpegArgs) `
    -WorkingDirectory $Root `
    -WindowStyle Hidden `
    -RedirectStandardOutput $FfmpegOut `
    -RedirectStandardError $FfmpegErr `
    -PassThru

Start-Sleep -Seconds 2
if ($ffmpegProc.HasExited) {
    Write-Warn "FFmpeg exited immediately. Check $FfmpegErr."
    if (Test-Path $FfmpegErr) { Get-Content -Path $FfmpegErr }
    Stop-Process -Id $mediaMtxProc.Id -Force -ErrorAction SilentlyContinue
    exit 2
}

@{
    mediaMtxPid = $mediaMtxProc.Id
    ffmpegPid = $ffmpegProc.Id
    rtspUrl = $rtspViewUrl
    cameraName = $CameraName
    startedAt = (Get-Date).ToString("o")
} | ConvertTo-Json | Set-Content -Path $PidFile -Encoding UTF8

Write-Ok "RTSP camera stream is ready: $rtspViewUrl"
exit 0
