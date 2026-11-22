@echo off
title RoboForge Studio
cd /d "%~dp0"
if exist ".venv\Scripts\pythonw.exe" (
    start "" ".venv\Scripts\pythonw.exe" "desktop\app_desktop.py"
) else (
    ".venv\Scripts\python.exe" "desktop\app_desktop.py"
)
