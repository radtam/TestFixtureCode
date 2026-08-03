@echo off
title Cycle Test Project Tracker
cd /d "%~dp0"
echo Starting the Cycle Test Project Tracker...
echo.
python server.py
if errorlevel 1 (
  echo.
  echo The tracker could not start. Confirm that Python is installed, then try again.
  pause
)
