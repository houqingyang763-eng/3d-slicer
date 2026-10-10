@echo off
set "CLASSROOM_ROOT=%~dp0"
start "" "%CLASSROOM_ROOT%tools\Slicer-5.12.4\Slicer.exe" --no-splash --python-script "%CLASSROOM_ROOT%scripts\start_ct.py"
