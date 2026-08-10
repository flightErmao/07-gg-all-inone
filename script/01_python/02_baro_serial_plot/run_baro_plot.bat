@echo off
setlocal
cd /d "%~dp0"
py -3.12 "%~dp0baro_serial_plot.py"
if errorlevel 1 (
    echo.
    echo GUI start failed. Install dependencies with:
    echo py -3.12 -m pip install -r "%~dp0requirements.txt"
    pause
)
