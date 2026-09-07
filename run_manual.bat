@echo off
cd /d "%~dp0"
python run_robot_bird.py
if errorlevel 1 pause
