@echo off
rem HV_control first power-up GUI. Uses the Anaconda base python (has tkinter, pyserial, matplotlib).
cd /d "%~dp0"
set PY=%USERPROFILE%\anaconda3\python.exe
if not exist "%PY%" set PY=%USERPROFILE%\Anaconda3\python.exe
if not exist "%PY%" set PY=python
"%PY%" -X utf8 hv_bringup_gui.py
if errorlevel 1 pause
