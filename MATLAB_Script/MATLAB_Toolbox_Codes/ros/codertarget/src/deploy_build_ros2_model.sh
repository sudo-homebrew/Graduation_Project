#!/bin/bash
#
# Copyright 2014-2021 The MathWorks, Inc.

# Exit shell script if any subcommand returns a non-zero status
set -e

# Set the default POSIX / C locale for all locale categories.
# This ensures that display characters in stdout and stderr are always single
# bytes and the standard ASCII character set is used.
export LC_ALL=C

ARCHIVE="$1"
ROS2_WS="$2"

echoErr() { 
   echo "$@" 1>&2; 
}

commandUsage() {
   echo "Usage: $(basename $0) ARCHIVE_NAME... ROS2_WS..." $1
   echo "Extract and build a C++ ROS node generated from a Simulink model." $1
   echo "ARCHIVE_NAME is the name of the TGZ file generated from the Simulink model." $1
   echo "ROS2_WS is the full path to your ROS 2 workspace." $1 
   echo "" $1
   echo "Example:" $1 
   echo "  ./$(basename $0) simulinkmodel.tgz ~/ros2_ws" $1
}

ros2workspaceHelp() {
   echo "" $1
   echo "You can create a ROS 2 workspace as follows:" $1
   echo "  mkdir -p ~/ros2_ws/src" $1
   echo "  cd ~/ros_ws" $1
   echo "  colcon build" $1
}


fullUsage() {
   commandUsage $1
   ros2workspaceHelp $1
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

if [ -z "$1" ] || ([ ! -z "$1" ] && [ "$1" == "-h" ] || [ "$1" == "--help" ]) ; then
   fullUsage
   exit 0
fi

if [ ! $# -eq 2 ] ; then
   echoErr "Expected two input arguments. Got $#."
   fullUsage 1>&2
   exit 1
fi

# Check ROS 2 workspace
if [ ! -d "$ROS2_WS" ] ; then
   echoErr "The ROS 2 workspace directory, "$ROS2_WS", does not exist."
   echoErr "Enter a valid ROS 2 workspace directory."
   ros2workspaceHelp 1>&2
   exit 1
fi

# Sanity check for ROS 2 workspace
if [ ! -d "$ROS2_WS"/src ] ; then
   echoErr "The ROS 2 workspace directory, "$ROS2_WS", is not a valid ROS 2 workspace."
   echoErr "Enter a valid ROS 2 workspace directory."
   ros2workspaceHelp 1>&2
   exit 1
fi

# Check Simulink archive
if [ ! -f "$ARCHIVE" ] ; then
   echoErr "The archive, "$ARCHIVE", does not exist."
   echoErr "Enter a valid Simulink model archive (.tgz file)."
   echoErr ""
   commandUsage 1>&2
   exit 1
fi

# Enforce that $ARCHIVE ends with .tgz, since the model 
# name is derived by stripping off the .tgz extension
if [ ${ARCHIVE: -4} != ".tgz" ] ; then
   echoErr "The archive, "$ARCHIVE", does not have a .tgz extension."
   echoErr "Enter a valid Simulink model archive (.tgz file)."
   echoErr ""   
   commandUsage 1>&2
   exit 1
fi

# Check if $ARCHIVE is a valid zip file
gzip -t "$ARCHIVE" 2> /dev/null
VALID_ZIP=$?
if [ $VALID_ZIP -ne 0 ] ; then
   echoErr "The archive, "$ARCHIVE", is not a valid .tgz (tar zip) file."
   echoErr ""
   commandUsage 1>&2
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
PROJECT_DIR="$ROS2_WS/src"

echo "ROS 2 project directory: $PROJECT_DIR"

# Extract files of the main archive (top-level model) to ROS 2 project directory
mkdir -p "$PROJECT_DIR"
rm -fr "$PROJECT_DIR/$PKGNAME"
tar -C "$PROJECT_DIR" -xf "$ARCHIVE"
rm -f "$ARCHIVE"

# Extract model reference archives (if needed)
MODEL_REF_LIST="$ARCHIVE_DIR/$ARCHIVE_BASE"ModelRefs.txt
if [ -f "$MODEL_REF_LIST" ] ; then
    mdlRefPkgs=()
    while IFS= read -r mdlRefArchive
    do
        # Trim whitespaces and newlines to account for OS-specific text
        mdlRefArchive=$(trim "$ARCHIVE_DIR/$mdlRefArchive")

        echo "Extracting model reference archive $mdlRefArchive"        

        # Extract archive if it exists
        if [ -f "$mdlRefArchive" ] ; then
            # Create folder
            MDLREF_DIR="$PROJECT_DIR/$(toLowerCase $(basename "$mdlRefArchive" .tgz))"
            rm -fr "$MDLREF_DIR"
            # Extract archive into created folder
            tar -C "$PROJECT_DIR" -xf "$mdlRefArchive"
        fi
    done < "$MODEL_REF_LIST"
fi

# Ensure that catkin_make will rebuild the executable
touch "$PROJECT_DIR/$PKGNAME"/src/*.cpp

# Build the Simulink model as a ROS 2 colcon project
# Ignore error code from the source command. If the environment setup is
# problematic, catkin_make will fail.
source "$ROS2_WS"/install/setup.bash || true
CURR_DIR=`pwd`
cd "$ROS2_WS"
colcon build --packages-up-to "$PKGNAME"
cd "$CURR_DIR"

exit 0
