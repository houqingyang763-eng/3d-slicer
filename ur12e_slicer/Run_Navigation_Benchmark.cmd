@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0Run_Navigation_Benchmark.ps1"
if errorlevel 1 pause
