@echo off
chcp 65001 >nul
cd /d "%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0one_click_start.ps1"
if errorlevel 1 (
    echo.
    echo [ERROR] One Click Start failed. Check the messages above.
    pause
    exit /b %errorlevel%
)
