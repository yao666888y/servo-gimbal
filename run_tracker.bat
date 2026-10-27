@echo off
rem ===========================================================================
rem  Vision tracker launcher (ASCII only: cmd.exe parses .bat in OEM codepage)
rem
rem  Usage (from any directory):
rem     G:\servo-gimbal\run_tracker.bat --list
rem     G:\servo-gimbal\run_tracker.bat --mode sim --port COM7
rem     G:\servo-gimbal\run_tracker.bat --port COM7 --cam 0 --mode color
rem
rem  Double-click (no args) -> shows help and keeps the window open.
rem ===========================================================================

cd /d "%~dp0"

set "PY=D:\anaconda3\envs\vision_servo\python.exe"
if not exist "%PY%" set "PY=python"

echo [python] %PY%
echo [cwd]    %CD%
echo.

if "%~1"=="" (
    "%PY%" "tools\vision_tracker.py" --help
    echo.
    pause
) else (
    "%PY%" "tools\vision_tracker.py" %*
)
