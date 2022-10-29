@echo off

:: Copyright 2019 The MathWorks, Inc.

:: echo All params: %* 

SET VCVARS_CMD=%1
SET AMENT_PREFIX_PATH=%2
SET PYTHON_VENV_PATH=%3

:: Remove quotes so can add quotes where needed
for /f "delims=" %%i in ("%AMENT_PREFIX_PATH%") do set "AMENT_PREFIX_PATH=%%~i"
for /f "delims=" %%i in ("%PYTHON_VENV_PATH%") do set "PYTHON_VENV_PATH=%%~i"

:: Activate local python
call "%PYTHON_VENV_PATH%\Scripts\activate"

:: Call vcvars
call %VCVARS_CMD% x86_amd64

:: Set the env variables as it would be from local_setup.bash
SET PATH=%AMENT_PREFIX_PATH%\Scripts;%AMENT_PREFIX_PATH%\bin;%PATH%
SET PYTHONPATH=%AMENT_PREFIX_PATH%\lib\site-packages;%PYTHON_VENV_PATH%\lib\site-packages
SET PKG_CONFIG_PATH="%AMENT_PREFIX_PATH%\lib\pkgconfig"
SET CMAKE_PREFIX_PATH=%AMENT_PREFIX_PATH%
SET COLCON_PREFIX_PATH=%AMENT_PREFIX_PATH%
