@echo off

:: Copyright 2020-2021 The MathWorks, Inc.

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

:: Set the env variables as it would be from local_setup.bash
SET PATH=%CATKIN_PREFIX_PATH%\Scripts;%CATKIN_PREFIX_PATH%\bin;%CATKIN_PREFIX_PATH%\console_bridge\bin;%CATKIN_PREFIX_PATH%\tinyxml2\bin;%PYTHON_VENV_PATH%;%PATH%
SET PYTHONPATH=%CATKIN_PREFIX_PATH%\lib\site-packages;%PYTHON_VENV_PATH%\lib\site-packages
SET PKG_CONFIG_PATH="%CATKIN_PREFIX_PATH%\lib\pkgconfig"
SET ROS1_INSTALL_DIR=%CATKIN_PREFIX_PATH%
SET ROSOUT_DISABLE_FILE_LOGGING=True
SET ROS_MASTER_URI=http://%ROS_MASTER_IP_ADDRESS%:%ROS_MASTER_IP_PORT%
SET ROS_IP=%ROS_MASTER_IP_ADDRESS%
SET ROS_HOME=%Temp%\.ros

echo Launching Ros-Master with following setup :
echo ROS_MASTER_URI = %ROS_MASTER_URI%
echo ROS_IP = %ROS_IP%
echo ROS_HOME = %ROS_HOME%

REM instead of generating a temp.bat file and running, set necessary env variables directly 
set CMAKE_PREFIX_PATH=%CATKIN_PREFIX_PATH:"=%
set LD_LIBRARY_PATH=%CMAKE_PREFIX_PATH%\lib;%LD_LIBRARY_PATH%
set PATH=%CMAKE_PREFIX_PATH%\bin;%CMAKE_PREFIX_PATH%\lib;%PATH%
set PKG_CONFIG_PATH=%CMAKE_PREFIX_PATH%\lib\pkgconfig;%PKG_CONFIG_PATH%
set PYTHONPATH=%CMAKE_PREFIX_PATH%\lib/site-packages;%PYTHONPATH%

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
