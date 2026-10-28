@echo off
setlocal
cd /d "%~dp0"
set "PY_EXE=%USERPROFILE%\miniconda3\python.exe"
if not exist "%PY_EXE%" (
    echo Python not found: %PY_EXE%
    pause
    exit /b 1
)
"%PY_EXE%" network_dashboard.py
