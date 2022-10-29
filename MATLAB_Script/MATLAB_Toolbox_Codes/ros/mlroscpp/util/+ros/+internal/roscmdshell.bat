@echo off

:: Copyright 2019-2020 The MathWorks, Inc.

:: echo All params: %* 

SET VCVARS_CMD=%1
SET CATKIN_PREFIX_PATH=%2
SET PYTHON_VENV_PATH=%3

:: Remove quotes so can add quotes where needed
for /f "delims=" %%i in ("%CATKIN_PREFIX_PATH%") do set "CATKIN_PREFIX_PATH=%%~si"
for /f "delims=" %%i in ("%PYTHON_VENV_PATH%") do set "PYTHON_VENV_PATH=%%~si"

:: Activate local python
call "%PYTHON_VENV_PATH%\Scripts\activate"

:: Call vcvars
call %VCVARS_CMD% x86_amd64

call "%~dp0\rossetup.bat"