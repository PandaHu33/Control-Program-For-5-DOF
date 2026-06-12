@echo off
chcp 65001 >nul
cd /d "%~dp0"

powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0run_vr_static_server.ps1" -DeviceId "%~1"
if errorlevel 1 (
    echo.
    echo [ERROR] VR static server startup failed.
    pause
    exit /b %errorlevel%
)
