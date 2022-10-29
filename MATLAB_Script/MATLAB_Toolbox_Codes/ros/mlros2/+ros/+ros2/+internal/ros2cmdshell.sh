# /usr/bin/env bash -e login

#   Copyright 2019-2021 The MathWorks, Inc.

# to debug uncomment the following line
# set -x
# echo $@

export AMENT_PREFIX_PATH=$1
export LOCAL_PYTHON_VENV_PATH=$2
if [ -z "$AMENT_PREFIX_PATH" ]; then
    export AMENT_PREFIX_PATH=REPLACE_AMENT_PREFIX_PATH
fi
if [ -z "$AMENT_PREFIX_PATH" ]; then
    echo "No AMENT_PREFIX_PATH found"
    exit 1
fi
if [ -z "$LOCAL_PYTHON_VENV_PATH" ]; then
    export LOCAL_PYTHON_VENV_PATH=REPLACE_LOCAL_PYTHON_VENV_PATH
fi
if [ -z "$LOCAL_PYTHON_VENV_PATH" ]; then
    echo "No LOCAL_PYTHON_VENV_PATH found"
    exit 1
fi


# Activate local python
source $LOCAL_PYTHON_VENV_PATH/bin/activate

# find in linux has a nice -printf verb but mac does not support that
# hence the workaround
sitepackages=`find $LOCAL_PYTHON_VENV_PATH/lib -type d -name site-packages -print`
sitepaths=`for folders in $sitepackages; do echo $folders:;done`

# Set the env variables as it would be from local_setup.bash
export LD_LIBRARY_PATH=$AMENT_PREFIX_PATH/lib:$LD_LIBRARY_PATH
export DYLD_LIBRARY_PATH=$AMENT_PREFIX_PATH/lib:$DYLD_LIBRARY_PATH
export PATH=$AMENT_PREFIX_PATH/bin:$LOCAL_PYTHON_VENV_PATH/bin:$PATH
export PYTHONPATH=$AMENT_PREFIX_PATH/lib/python3.9/site-packages:$sitepaths$PYTHONPATH
export PKG_CONFIG_PATH=$AMENT_PREFIX_PATH/lib/pkgconfig
export CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH
export COLCON_PREFIX_PATH=$AMENT_PREFIX_PATH

# source $LOCAL_AMENT_PREFIX_PATH/local_setup.bash

if [ "$ROS_DOMAIN_ID" = "" ]; then
    unset ROS_DOMAIN_ID
fi
