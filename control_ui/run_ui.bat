@echo off
cd /d "%~dp0"

echo [INFO] Testing SSH passwordless login...
ssh -o BatchMode=yes -o ConnectTimeout=5 -o UserKnownHostsFile=NUL -o StrictHostKeyChecking=no -o LogLevel=ERROR night@192.168.1.35 "echo ok" >nul 2>nul

if errorlevel 1 (
    echo [WARN] SSH passwordless login failed. Running first-connect setup...
    powershell -ExecutionPolicy Bypass -File "%~dp0setup_ssh_first_connect.ps1" -User night -HostIp 192.168.1.35

    if errorlevel 1 (
        echo [ERROR] SSH setup failed.
        pause
        exit /b 1
    )
)

echo [INFO] Starting main program...
call "%~dp0one_click_start.bat"
