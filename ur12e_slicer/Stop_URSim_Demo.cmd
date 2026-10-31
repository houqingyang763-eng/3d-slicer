@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0URSim_Windows.ps1" -Action stop %*
pause
