#!/bin/bash
#
# Copyright 2020-2021 The MathWorks, Inc.

ARCHIVE="$1"
ROS2_WS="$2"

ros2WorkspaceHelp() {
   echo ""
   echo "You can create a ROS 2 workspace as follows:"
   echo "  mkdir -p ~/ros2_ws/src"
}


commandUsage() {
   echo "Usage: $(basename $0) ARCHIVE_NAME... ROS2_WS..."
   echo "Extract and build a C++ ROS 2 node generated from a Simulink model."
   echo "ARCHIVE_NAME is the name of the TGZ file generated from the Simulink model."
   echo "ROS2_WS is the full path to your ROS 2 workspace."
   echo ""
   echo "Example:"
   echo "  ./$(basename $0) simulinkmodel.tgz ~/ros2_ws"   
}


fullUsage() {
   commandUsage
   ros2WorkspaceHelp
   exit
}


toLowerCase() {
   echo $1 | tr '[A-Z]' '[a-z]'
}

trim() {
    local var="$*"
    # remove leading whitespace characters
    var="${var#"${var%%[![:space:]]*}"}"
    # remove trailing whitespace characters
    var="${var%"${var##*[![:space:]]}"}"
    echo -n "$var"
}

if [ -z "$1" ] || ([ ! -z "$1" ] && [ "$1" = "-h" ] || [ "$1" = "--help" ]) ; then
   fullUsage
   exit 0
fi

if [ ! $# -eq 2 ] ; then
   echo "Expected two input arguments. Got $#."
   fullUsage
   exit 1
fi

# Check ROS 2 workspace
if [ ! -d "$ROS2_WS" ] ; then
   echo "The ROS 2 workspace directory, "$ROS2_WS", does not exist."
   echo "Enter a valid ROS 2 workspace directory."
   exit 1
fi

# Check Simulink archive
if [ ! -f "$ARCHIVE" ] ; then
   echo "The archive, "$ARCHIVE", does not exist."
   echo "Enter a valid Simulink model archive (.tgz file)."
   echo ""
   commandUsage
   exit 1
fi

# Enforce that $ARCHIVE ends with .tgz, since the model 
# name is derived by stripping off the .tgz extension
if [ ${ARCHIVE: -4} != ".tgz" ] ; then
   echo "The archive, "$ARCHIVE", does not have a .tgz extension."
   echo "Enter a valid Simulink model archive (.tgz file)."
   echo ""   
   commandUsage
   exit 1
fi

# Check if $ARCHIVE is a valid zip file
gzip -t "$ARCHIVE" 2> /dev/null
VALID_ZIP=$?
if [ $VALID_ZIP -ne 0 ] ; then
   echo "The archive, "$ARCHIVE", is not a valid .tgz (tar zip) file."
   echo ""
   commandUsage
   exit 1   
fi

# Check for one of the standard files generated from Simulink
# (ros2nodeinterface.cpp)
tar ztf "$ARCHIVE" | grep -q -E -- 'main|ros2nodeinterface'.cpp 2> /dev/null
VALID_CODEGEN_ARCHIVE=$?
if [ $VALID_CODEGEN_ARCHIVE -ne 0 ] ; then
   echo "The archive, "$ARCHIVE", is not a valid archive (.tgz file) of files generated from Simulink/MATLAB Coder."
   echo ""
   commandUsage
   exit 1
fi

# $ARCHIVE appears to be valid.
# Extract and build it

ARCHIVE_DIR=$(dirname "$ARCHIVE")
ARCHIVE_BASE=$(basename "$ARCHIVE" .tgz)

PKGNAME=$(toLowerCase $ARCHIVE_BASE)
PROJECT_DIR="$ROS2_WS/src/$PKGNAME"

echo "ROS 2 project directory: $PROJECT_DIR"

# Extract files of the main archive (top-level model) to ROS 2 project directory
mkdir -p "$PROJECT_DIR"
rm -fr "$PROJECT_DIR"
tar -C "$ROS2_WS/src" -xf "$ARCHIVE"

# Ensure that colcon build will rebuild the executable
touch "$PROJECT_DIR"/*.cpp

# Build the Simulink model as a ROS 2 project
CURR_DIR=`pwd`
cd "$ROS2_WS"
# Build all the dependent packages with the ROS 2 node package
colcon build --packages-up-to "$PKGNAME"
cd "$CURR_DIR"

exit 0
