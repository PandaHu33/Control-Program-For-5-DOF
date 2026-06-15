$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
$PidFile = Join-Path $Root "logs\rtsp_camera_pids.json"

function Write-Info($Text) { Write-Host "[INFO] $Text" -ForegroundColor Cyan }
function Write-Ok($Text) { Write-Host "[OK] $Text" -ForegroundColor Green }
function Write-Warn($Text) { Write-Host "[WARN] $Text" -ForegroundColor Yellow }

if (-not (Test-Path $PidFile)) {
    exit 0
}

try {
    $pids = Get-Content -Path $PidFile -Raw | ConvertFrom-Json
    foreach ($id in @($pids.ffmpegPid, $pids.mediaMtxPid)) {
        if (-not $id) { continue }
        $proc = Get-Process -Id $id -ErrorAction SilentlyContinue
        if ($proc) {
            Write-Info ("Stopping RTSP camera helper pid={0} process={1}" -f $id, $proc.ProcessName)
            Stop-Process -Id $id -Force -ErrorAction SilentlyContinue
        }
    }
    Remove-Item -LiteralPath $PidFile -Force -ErrorAction SilentlyContinue
    Write-Ok "RTSP camera stream stopped."
} catch {
    Write-Warn ("Failed to stop RTSP camera stream: {0}" -f $_.Exception.Message)
}
