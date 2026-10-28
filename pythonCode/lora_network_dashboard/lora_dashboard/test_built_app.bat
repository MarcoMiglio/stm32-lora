@echo off
setlocal
cd /d "%~dp0"
if not exist "dist\LoRaDashboard\LoRaDashboard.exe" (
    echo Build not found. Run build_windows.bat first.
    pause
    exit /b 1
)
start "" "dist\LoRaDashboard\LoRaDashboard.exe"
