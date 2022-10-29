# /usr/bin/env bash -e login

#   Copyright 2019 The MathWorks, Inc.

# to debug uncomment the following line
# set -x
# echo $@

export CATKIN_PREFIX_PATH=$1
export LOCAL_PYTHON_VENV_PATH=$2
export MATLAB_BIN=$3

if [ -z "$CATKIN_PREFIX_PATH" ]; then
    export CATKIN_PREFIX_PATH=REPLACE_CATKIN_PREFIX_PATH
fi
if [ -z "$CATKIN_PREFIX_PATH" ]; then
    echo "No CATKIN_PREFIX_PATH found"
    exit 1
fi
if [ -z "$LOCAL_PYTHON_VENV_PATH" ]; then
    export LOCAL_PYTHON_VENV_PATH=REPLACE_LOCAL_PYTHON_VENV_PATH
fi
if [ -z "$LOCAL_PYTHON_VENV_PATH" ]; then
    echo "No LOCAL_PYTHON_VENV_PATH found"
    exit 1
fi

export MATLAB_BIN=REPLACE_MATLABBIN_PATH

# Activate local python
source $LOCAL_PYTHON_VENV_PATH/bin/activate

source "REPLACE_ROSSETUP_PATH/rossetup.sh"
