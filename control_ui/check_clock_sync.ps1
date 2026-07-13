param(
    [string]$HostName = "192.168.1.35",
    [string]$User = "night",
    [int]$Samples = 20,
    [switch]$Sync
)

$ErrorActionPreference = "Stop"

if ($Sync) {
    & w32tm /resync | Out-Host
    & ssh -o BatchMode=yes -o ConnectTimeout=5 "$User@$HostName" "sudo -n timedatectl set-ntp true"
    if ($LASTEXITCODE -ne 0) {
        throw "Jetson NTP enable failed"
    }
}

$measurements = @()
for ($i = 0; $i -lt [Math]::Max(1, $Samples); $i++) {
    $t0 = [DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds()
    $remoteText = (& ssh -o BatchMode=yes -o ConnectTimeout=5 "$User@$HostName" "date +%s%3N").Trim()
    $t1 = [DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds()
    if ($LASTEXITCODE -ne 0 -or $remoteText -notmatch '^\d+$') {
        throw "Unable to read Jetson clock"
    }
    $remote = [double]$remoteText
    $midpoint = ($t0 + $t1) / 2.0
    $measurements += [pscustomobject]@{
        offset_ms = $remote - $midpoint
        uncertainty_ms = ($t1 - $t0) / 2.0
        rtt_ms = $t1 - $t0
    }
}

$best = $measurements | Sort-Object uncertainty_ms | Select-Object -First 1
$best | Format-List
if ([Math]::Abs($best.offset_ms) + $best.uncertainty_ms -gt 1.0) {
    Write-Warning "Clock residual/uncertainty is above 1 ms; do not treat cross-host one-way latency as exact."
    exit 2
}
Write-Output "CLOCK_SYNC_OK"
