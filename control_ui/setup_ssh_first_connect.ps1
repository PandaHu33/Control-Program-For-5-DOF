param(
    [string]$User = "mumu",
    [string]$HostIp = "192.168.1.35"
)

Write-Host "========================================"
Write-Host " Jetson SSH First Connect Setup"
Write-Host " Target: $User@$HostIp"
Write-Host "========================================"

$sshDir = Join-Path $env:USERPROFILE ".ssh"
$keyPath = Join-Path $sshDir "id_ed25519"
$pubKeyPath = "$keyPath.pub"

# 1. Create .ssh folder
if (!(Test-Path $sshDir)) {
    Write-Host "[INFO] Creating local .ssh folder..."
    New-Item -ItemType Directory -Path $sshDir | Out-Null
}

# 2. Generate SSH key if missing
if (!(Test-Path $keyPath) -or !(Test-Path $pubKeyPath)) {
    Write-Host "[INFO] SSH key not found. Generating new ed25519 key..."
    ssh-keygen -t ed25519 -f $keyPath -N ""
    if ($LASTEXITCODE -ne 0) {
        Write-Host "[ERROR] Failed to generate SSH key."
        exit 1
    }
} else {
    Write-Host "[INFO] SSH key already exists."
}

# 3. Remove old host key to avoid Host key verification failed
Write-Host "[INFO] Removing old known_hosts entry for $HostIp..."
ssh-keygen -R $HostIp | Out-Null

# 4. Accept host key once
Write-Host "[INFO] Accepting SSH host key..."
ssh -o StrictHostKeyChecking=accept-new -o ConnectTimeout=5 "$User@$HostIp" "echo host-key-ok"

if ($LASTEXITCODE -ne 0) {
    Write-Host "[ERROR] Cannot connect to $User@$HostIp."
    Write-Host "请检查："
    Write-Host "  1. 板卡是否开机"
    Write-Host "  2. 网线是否连接"
    Write-Host "  3. 板卡 IP 是否是 $HostIp"
    Write-Host "  4. SSH 服务是否启动"
    exit 2
}

# 5. Copy public key to Jetson
Write-Host "[INFO] Installing public key to Jetson..."
Write-Host "[INFO] 这一步会要求输入一次板卡 mumu 用户密码。"

Get-Content $pubKeyPath | ssh -o StrictHostKeyChecking=accept-new "$User@$HostIp" "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 700 ~/.ssh && chmod 600 ~/.ssh/authorized_keys"

if ($LASTEXITCODE -ne 0) {
    Write-Host "[ERROR] Failed to install public key."
    exit 3
}

# 6. Test passwordless SSH
Write-Host "[INFO] Testing passwordless SSH..."

ssh -o BatchMode=yes -o ConnectTimeout=5 "$User@$HostIp" "echo ok"

if ($LASTEXITCODE -eq 0) {
    Write-Host "========================================"
    Write-Host "[SUCCESS] SSH 免密配置成功。"
    Write-Host "后续可以直接使用："
    Write-Host "ssh -o BatchMode=yes -o ConnectTimeout=5 $User@$HostIp `"echo ok`""
    Write-Host "========================================"
    exit 0
} else {
    Write-Host "[ERROR] SSH key installed, but passwordless login failed."
    Write-Host "可能原因："
    Write-Host "  1. 板卡密码输错"
    Write-Host "  2. authorized_keys 权限异常"
    Write-Host "  3. 板卡 sshd 禁用了公钥登录"
    exit 4
}
