@echo off
title Cycle Test Code Tracker
cd /d "%~dp0"
echo Starting the Cycle Test Code Tracker...
echo.
python server.py
if errorlevel 1 (
  echo.
  echo The tracker could not start. Confirm that Python is installed, then try again.
  pause
)
