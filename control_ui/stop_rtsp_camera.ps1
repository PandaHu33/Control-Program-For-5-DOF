$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
$PidFile = Join-Path $Root "logs\rtsp_camera_pids.json"
$LatestFrameStatus = Join-Path $Root "logs\latest_frame_stream_status.json"
$RootText = $Root.ToLowerInvariant()

function Write-Info($Text) { Write-Host "[INFO] $Text" -ForegroundColor Cyan }
function Write-Ok($Text) { Write-Host "[OK] $Text" -ForegroundColor Green }
function Write-Warn($Text) { Write-Host "[WARN] $Text" -ForegroundColor Yellow }

function Stop-RtspProcessById($Id) {
    if (-not $Id) { return $false }
    $proc = Get-Process -Id $Id -ErrorAction SilentlyContinue
    if (-not $proc) { return $false }
    Write-Info ("Stopping RTSP camera helper pid={0} process={1}" -f $Id, $proc.ProcessName)
    Stop-Process -Id $Id -Force -ErrorAction SilentlyContinue
    return $true
}

function Stop-OrphanedRtspProcesses {
    $stopped = 0
    foreach ($name in @("ffmpeg.exe", "mediamtx.exe", "python.exe", "pythonw.exe")) {
        foreach ($proc in @(Get-CimInstance Win32_Process -Filter "name='$name'" -ErrorAction SilentlyContinue)) {
            $cmd = [string]$proc.CommandLine
            if (-not $cmd) { continue }
            $lowerCmd = $cmd.ToLowerInvariant()
            if ($lowerCmd.Contains($RootText) -or $lowerCmd.Contains("latest_frame_dual_stream.py")) {
                Write-Info ("Stopping orphan RTSP helper pid={0} process={1}" -f $proc.ProcessId, $name)
                Stop-Process -Id $proc.ProcessId -Force -ErrorAction SilentlyContinue
                $stopped += 1
            }
        }
    }
    return $stopped
}

try {
    $stopped = 0
    if (Test-Path $PidFile) {
        $pids = Get-Content -Path $PidFile -Raw | ConvertFrom-Json
        foreach ($id in @($pids.ffmpegPid, $pids.capturePid, $pids.encoderPid, $pids.mediaMtxPid)) {
            if (Stop-RtspProcessById $id) {
                $stopped += 1
            }
        }
        Remove-Item -LiteralPath $PidFile -Force -ErrorAction SilentlyContinue
    }
    if (Test-Path $LatestFrameStatus) {
        try {
            $status = Get-Content -Path $LatestFrameStatus -Raw | ConvertFrom-Json
            foreach ($id in @($status.pythonPid, $status.ffmpegPid)) {
                if (Stop-RtspProcessById $id) {
                    $stopped += 1
                }
            }
        } catch {}
        Remove-Item -LiteralPath $LatestFrameStatus -Force -ErrorAction SilentlyContinue
    }
    $stopped += Stop-OrphanedRtspProcesses
    if ($stopped -gt 0) {
        Write-Ok ("RTSP camera stream stopped. helpers={0}" -f $stopped)
    } else {
        Write-Ok "RTSP camera stream was not running."
    }
} catch {
    Write-Warn ("Failed to stop RTSP camera stream: {0}" -f $_.Exception.Message)
    exit 1
}
