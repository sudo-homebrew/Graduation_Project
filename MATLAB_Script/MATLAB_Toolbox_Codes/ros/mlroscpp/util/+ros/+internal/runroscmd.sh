#! /usr/bin/env bash

#   Copyright 2019 The MathWorks, Inc.

# to debug uncomment the following line
# set -x
# echo $@

export UNUSED=$1
export CATKIN_PREFIX_PATH=$2
export LOCAL_PYTHON_VENV_PATH=$3

# Activate local python
source $LOCAL_PYTHON_VENV_PATH/bin/activate

export SCRIPT_DIR=`dirname ${BASH_SOURCE[0]}`
source $SCRIPT_DIR/rossetup.sh 

# source local_setup.bash if it exists
if [ -f "local_setup.bash" ]; then
	. "local_setup.bash"
fi

#skip the first argument and evaluate all the rest
shift 
shift
shift
shift
$@