@echo off

:: Copyright 2019-2021 The MathWorks, Inc.

:: echo All params: %* 

SET VCVARS_CMD=%1
SET CATKIN_PREFIX_PATH=%2
SET PYTHON_VENV_PATH=%3

:: Remove quotes so can add quotes where needed
for /f "delims=" %%i in ("%CATKIN_PREFIX_PATH%") do set "CATKIN_PREFIX_PATH=%%~si"
for /f "delims=" %%i in ("%PYTHON_VENV_PATH%") do set "PYTHON_VENV_PATH=%%~si"


:: Activate local python
call "%PYTHON_VENV_PATH%\Scripts\activate"

:: Call extra setup
if NOT %VCVARS_CMD% == " " (
    call %VCVARS_CMD% x86_amd64
)

call "%~dp0\rossetup.bat"

:: Call local_setup.bat if it exists
if exist local_setup.bat ( 
  call local_setup.bat
)

:: Filter out the upto 3 parameters
SET _all=%*
call SET ACTUAL_CMD_AND_PARAM=%%_all:*%4 %5=%%
SET ACTUAL_CMD_AND_PARAM=%4 %5%ACTUAL_CMD_AND_PARAM%

:: echo %ACTUAL_CMD_AND_PARAM%
:: pwd
%ACTUAL_CMD_AND_PARAM%
