# Copyright 2019-2021 The MathWorks, Inc. 
# find in linux has a nice -printf verb but mac does not support that
# hence the workaround
sitepackages=`find $LOCAL_PYTHON_VENV_PATH/lib -type d -name site-packages -print`
sitepaths=`for folders in $sitepackages; do echo $folders:;done`

# Set the env variables as it would be from local_setup.bash
export LD_LIBRARY_PATH=$CATKIN_PREFIX_PATH/lib:$CATKIN_PREFIX_PATH/tinyxml2/lib:$CATKIN_PREFIX_PATH/console_bridge/lib:$CATKIN_PREFIX_PATH/lz4/lib:$CATKIN_PREFIX_PATH/bzip2/lib:$MATLAB_BIN:$LD_LIBRARY_PATH
export DYLD_LIBRARY_PATH=$CATKIN_PREFIX_PATH/lib:$CATKIN_PREFIX_PATH/tinyxml2/lib:$CATKIN_PREFIX_PATH/console_bridge/lib:$CATKIN_PREFIX_PATH/lz4/lib:$CATKIN_PREFIX_PATH/bzip2/lib:$MATLAB_BIN:$DYLD_LIBRARY_PATH
export PATH=$CATKIN_PREFIX_PATH/bin:$CATKIN_PREFIX_PATH/bzip2/bin:$LOCAL_PYTHON_VENV_PATH/bin:$MATLAB_BIN:$PATH
export PYTHONPATH=$CATKIN_PREFIX_PATH/lib/python3.9/site-packages:$sitepaths$PYTHONPATH
export PKG_CONFIG_PATH=$CATKIN_PREFIX_PATH/lib/pkgconfig
export CMAKE_PREFIX_PATH=$CATKIN_PREFIX_PATH
export CATKIN_PREFIX_PATH=$CATKIN_PREFIX_PATH
export ROS1_INSTALL_DIR=$CATKIN_PREFIX_PATH
export _CATKIN_SETUP_DIR=$CATKIN_PREFIX_PATH
export _PYTHON=`which python`
export ROSOUT_DISABLE_FILE_LOGGING=True

if [ -d /lib/x86_64-linux-gnu ]; then
   export LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
fi

source $_CATKIN_SETUP_DIR/setup.sh
