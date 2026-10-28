@echo off
setlocal
cd /d "%~dp0"

echo ========================================
echo LoRa Network Dashboard V1.10 - BUILD
echo Tkinter / Miniconda Tcl_Obj packaging fix
echo ========================================
echo.

set "PY_EXE=%USERPROFILE%\miniconda3\python.exe"

if not exist "%PY_EXE%" (
    echo ERROR: Python not found:
    echo   %PY_EXE%
    echo.
    echo Edit PY_EXE in this file if the development PC uses another Python.
    goto :fail
)

echo Using:
echo   %PY_EXE%
"%PY_EXE%" --version
if errorlevel 1 goto :fail

echo.
echo Checking Tkinter and pyserial...
"%PY_EXE%" -c "import tkinter, serial; print('Tkinter OK -', tkinter.TkVersion); print('pyserial OK -', serial.__version__)"
if errorlevel 1 goto :fail

echo.
echo Updating PyInstaller to a recent 6.x version...
"%PY_EXE%" -m pip install --upgrade "pyinstaller>=6.22.3,<7"
if errorlevel 1 goto :fail

echo.
echo Starting robust portable build...
"%PY_EXE%" build_portable.py
if errorlevel 1 goto :fail

echo.
echo You can now launch:
echo   dist\LoRaDashboard\LoRaDashboard.exe
echo.
pause
exit /b 0

:fail
echo.
echo ========================================
echo BUILD FAILED
echo ========================================
echo Copy the error shown above if you need help diagnosing it.
echo.
pause
exit /b 1
