#! /usr/bin/env bash

#   Copyright 2019-2021 The MathWorks, Inc.

# to debug uncomment the following line
# set -x

export AMENT_PREFIX_PATH=$1
export LOCAL_PYTHON_VENV_PATH=$2

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

#skip the first argument and evaluate all the rest
shift 
shift
$LOCAL_PYTHON_VENV_PATH/bin/colcon "$@"