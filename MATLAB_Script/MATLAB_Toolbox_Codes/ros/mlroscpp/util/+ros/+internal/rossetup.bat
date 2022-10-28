REM Local version modified to suit ROS Toolbox
:: Copyright 2019 The MathWorks, Inc.
:: Set the env variables as it would be from local_setup.bash
SET PATH=%CATKIN_PREFIX_PATH%\Scripts;%CATKIN_PREFIX_PATH%\bin;%CATKIN_PREFIX_PATH%\console_bridge\bin;%CATKIN_PREFIX_PATH%\tinyxml2\bin;%PYTHON_VENV_PATH%;%PATH%
SET PYTHONPATH=%CATKIN_PREFIX_PATH%\lib\site-packages;%PYTHON_VENV_PATH%\lib\site-packages
SET PKG_CONFIG_PATH="%CATKIN_PREFIX_PATH%\lib\pkgconfig"
SET CMAKE_PREFIX_PATH=%CATKIN_PREFIX_PATH%
SET CATKIN_PREFIX_PATH=%CATKIN_PREFIX_PATH%
SET ROS1_INSTALL_DIR=%CATKIN_PREFIX_PATH%
SET ROSOUT_DISABLE_FILE_LOGGING=True
:: SET ROS_MASTER_URI=http://%ROS_MASTER_IP_ADDRESS%:%ROS_MASTER_IP_PORT%
SET ROS_IP=%ROS_MASTER_IP_ADDRESS%
SET ROS_HOME=%Temp%\.ros

set _SETUP_UTIL=%CATKIN_PREFIX_PATH%\_setup_util.py
:: echo %_SETUP_UTIL%

if NOT EXIST "%_SETUP_UTIL%" (
  echo "Missing Python script: %_SETUP_UTIL%"
  exit 22
)

REM set the Python executable
set _PYTHON="%PYTHON_VENV_PATH%\Scripts\python.exe"

REM generate pseudo random temporary filename
:GenerateTempFilename
REM replace leading space of time with zero
set _SETUP_TMP=%Time: =0%
REM remove time delimiters
set _SETUP_TMP=%_SETUP_TMP::=%
set _SETUP_TMP=%_SETUP_TMP:.=%
set _SETUP_TMP=%_SETUP_TMP:,=%
set _SETUP_TMP=%_SETUP_TMP%%ROS_MATLAB_PID%
set _SETUP_TMP=%Temp%\setup.%_SETUP_TMP%.bat
if EXIST "%_SETUP_TMP%" do goto GenerateTempFilename
type NUL > "%_SETUP_TMP%"
if NOT EXIST "%_SETUP_TMP%" (
  echo "Could not create temporary file: %_SETUP_TMP%"
  exit 1
)

REM invoke Python script to generate necessary exports of environment variables
:: echo %_PYTHON% "%_SETUP_UTIL%" 
%_PYTHON% "%_SETUP_UTIL%" > "%_SETUP_TMP%"
if NOT EXIST "%_SETUP_TMP%" (
  echo "Could not create temporary file: %_SETUP_TMP%"
  return 1
)
:: type "%_SETUP_TMP%"
call "%_SETUP_TMP%"
del "%_SETUP_TMP%"
:: echo "%_SETUP_TMP%"

REM source all environment hooks
set _HOOK_COUNT=0
:hook_loop
if %_HOOK_COUNT% LSS %_CATKIN_ENVIRONMENT_HOOKS_COUNT% (
  REM set workspace for environment hook
  :: echo call set CATKIN_ENV_HOOK_WORKSPACE=%%_CATKIN_ENVIRONMENT_HOOKS_%_HOOK_COUNT%_WORKSPACE%%
  call set CATKIN_ENV_HOOK_WORKSPACE=%%_CATKIN_ENVIRONMENT_HOOKS_%_HOOK_COUNT%_WORKSPACE%%
  set _CATKIN_ENVIRONMENT_HOOKS_%_HOOK_COUNT%_WORKSPACE=

  REM call environment hook
  :: echo %%_CATKIN_ENVIRONMENT_HOOKS_%_HOOK_COUNT%%%
  call "%%_CATKIN_ENVIRONMENT_HOOKS_%_HOOK_COUNT%%%"
  set _CATKIN_ENVIRONMENT_HOOKS_%_HOOK_COUNT%=

  set CATKIN_ENV_HOOK_WORKSPACE=

  set /a _HOOK_COUNT=%_HOOK_COUNT%+1
  goto :hook_loop
)

REM unset temporary variables
set _SETUP_UTIL=
set _PYTHON=
set _PYTHONEXE=
set _PYTHON_FOUND=
set _SETUP_TMP=
set _CATKIN_ENVIRONMENT_HOOKS_COUNT=
set _HOOK_COUNT=