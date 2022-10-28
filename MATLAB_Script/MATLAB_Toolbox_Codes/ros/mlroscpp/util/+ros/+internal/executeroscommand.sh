#! /usr/bin/env bash

#   Copyright 2021 The MathWorks, Inc.

# to debug uncomment the following line
 set -x
 echo $@

export CATKIN_PREFIX_PATH=$1
export LOCAL_PYTHON_VENV_PATH=$2

# Activate local python
source $LOCAL_PYTHON_VENV_PATH/bin/activate

export SCRIPT_DIR=`dirname ${BASH_SOURCE[0]}`
source $SCRIPT_DIR/rossetup.sh 

# source local_setup.bash if it exists
if [ -f "local_setup.bash" ]; then
	. "local_setup.bash"
fi

#skip the first 2 arguments and evaluate all the rest
shift 2
eval "$*"