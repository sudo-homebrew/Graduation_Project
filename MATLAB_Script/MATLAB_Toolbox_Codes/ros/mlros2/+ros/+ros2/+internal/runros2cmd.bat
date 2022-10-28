@echo off

:: Copyright 2019 The MathWorks, Inc.

:: echo All params: %* 

SET AMENT_PREFIX_PATH=%1
SET PYTHON_VENV_PATH=%2

:: Remove quotes so can add quotes where needed
for /f "delims=" %%i in ("%AMENT_PREFIX_PATH%") do set "AMENT_PREFIX_PATH=%%~i"
for /f "delims=" %%i in ("%PYTHON_VENV_PATH%") do set "PYTHON_VENV_PATH=%%~i"

:: Activate local python
call "%PYTHON_VENV_PATH%\Scripts\activate"

:: Set the env variables as it would be from local_setup.bash
SET PATH=%AMENT_PREFIX_PATH%\Scripts;%AMENT_PREFIX_PATH%\bin;%PATH%
SET PYTHONPATH=%AMENT_PREFIX_PATH%\lib\site-packages;%PYTHON_VENV_PATH%\lib\site-packages
SET PKG_CONFIG_PATH="%AMENT_PREFIX_PATH%\lib\pkgconfig"
SET ROS2SCRIPT=%AMENT_PREFIX_PATH%\Scripts\ros2-script.py
SET CMAKE_PREFIX_PATH=%AMENT_PREFIX_PATH%
SET COLCON_PREFIX_PATH=%AMENT_PREFIX_PATH%

:: Call local_setup.bat if it exists
if exist local_setup.bat ( 
  call local_setup.bat
)

:: Filter out the upto 2 parameters
SET _all=%*
call SET ACTUAL_CMD_AND_PARAM=%%_all:*%2=%%
:: echo "%ROS2SCRIPT%" %ACTUAL_CMD_AND_PARAM%

python "%ROS2SCRIPT%" %ACTUAL_CMD_AND_PARAM%
