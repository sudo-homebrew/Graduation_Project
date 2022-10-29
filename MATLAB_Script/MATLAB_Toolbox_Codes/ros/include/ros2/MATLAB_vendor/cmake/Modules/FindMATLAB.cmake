#FindMATLAB.cmake
# Copyright 2018-2019 The MathWorks, Inc.
# The following variables are set if MATLAB is found. If MATLAB is not found, MATLAB_FOUND is set to false.
#  MATLAB_FOUND - True when MATLAB 
#  MATLAB_INCLUDE_DIRS
#  MATLAB_LIBRARY_DIR


# Set environment variable ${MATLAB_ROOT} that points to the root of where you ahve installed MATLAB

# Usage:
# In your CMakeLists.txt file do something like this:
# ...
# # MATLAB
# find_package(MATLAB REQUIRED)
# ...
# include_directories(${MATLAB_INCLUDE_DIRS})
# link_directories(${MATLAB_LIBRARY_DIR})
#
# In Windows, we make the assumption that, if the Poco files are installed, the default directory
# will be C:\MATLAB or C:\Program Files\MATLAB or C:\Programme\MATLAB.

message(STATUS "Searching for MATLAB installation...")

set(MATLAB_INCLUDE_PATH_DESCRIPTION
  "top-level directory containing the MATLAB include directories. E.g /usr/local/MATLAB/extern/include/ or c:\\MATLAB\\extern\\include\\")
set(MATLAB_INCLUDE_DIR_MESSAGE
  "Set the MATLAB_INCLUDE_DIR cmake cache entry to the ${MATLAB_INCLUDE_PATH_DESCRIPTION}")
set(MATLAB_LIBRARY_PATH_DESCRIPTION "top-level directory containing the MATLAB libraries.")
set(MATLAB_LIBRARY_DIR_MESSAGE
  "Set the MATLAB_LIBRARY_DIR cmake cache entry to the ${MATLAB_LIBRARY_PATH_DESCRIPTION}")

set(MATLAB_DIR_SEARCH $ENV{MATLAB_ROOT})
if(MATLAB_DIR_SEARCH)
  #message(STATUS "Searching in ${MATLAB_DIR_SEARCH}...")
  file(TO_CMAKE_PATH ${MATLAB_DIR_SEARCH} MATLAB_DIR_SEARCH)
endif()

# Look for an installation
find_path(MATLAB_INCLUDE_DIR
	matrix.h
	HINTS ${MATLAB_DIR_SEARCH}/extern/include
	DOC "The ${MATLAB_INCLUDE_PATH_DESCRIPTION}"
)

set (MATLAB_FOUND false)
if (MATLAB_INCLUDE_DIR)
	if (EXISTS ${MATLAB_INCLUDE_DIR}/matrix.h)
		set(MATLAB_INCLUDE_DIRS
			${MATLAB_INCLUDE_DIR}/
			${MATLAB_INCLUDE_DIR}/MatlabDataArray
			${MATLAB_DIR_SEARCH}/toolbox/ros/include/ros2)
	else()
		message(FATAL_ERROR "could not find extern/include/matrix.h...")
	endif()
else()
	message(FATAL_ERROR "Failed to find MATLAB/extern/include...set environment variable MATLAB_ROOT.")
endif()

if (WIN32)
	set(SUFFIX_FOR_LIBRARY_PATH win64)
elseif (APPLE)
	set(SUFFIX_FOR_LIBRARY_PATH maci64)
elseif (UNIX)
	set(SUFFIX_FOR_LIBRARY_PATH glnxa64)
else()
	message(FATAL_ERROR "Platform not supported")
endif()

#message (STATUS "SUFFIX_FOR_LIBRARY_PATH=${SUFFIX_FOR_LIBRARY_PATH}")
#message (STATUS "MATLAB_INCLUDE_DIR=${MATLAB_INCLUDE_DIR}")
#message (STATUS "MATLAB_DIR_SEARCH=${MATLAB_DIR_SEARCH}")
#message (STATUS "Prefix ${CMAKE_SHARED_LIBRARY_PREFIX} and Suffix = ${CMAKE_SHARED_LIBRARY_SUFFIX}")

if (EXISTS ${MATLAB_DIR_SEARCH}/extern/bin/${SUFFIX_FOR_LIBRARY_PATH}/libMatlabDataArray${CMAKE_SHARED_LIBRARY_SUFFIX})
	if (WIN32)
		set (MATLAB_FOUNDATION_LIB
			${MATLAB_DIR_SEARCH}/extern/lib/${SUFFIX_FOR_LIBRARY_PATH}/microsoft/libMatlabDataArray${CMAKE_SHARED_LIBRARY_SUFFIX}
			)
	else()
		set (MATLAB_FOUNDATION_LIB
			${MATLAB_DIR_SEARCH}/extern/bin/${SUFFIX_FOR_LIBRARY_PATH}/libMatlabDataArray${CMAKE_SHARED_LIBRARY_SUFFIX}
			)
	endif()
else()
	set (MATLAB_FOUNDATION_LIB "MATLAB_FOUNDATION_LIB-NOTFOUND")
endif()

if (MATLAB_FOUNDATION_LIB)
    #message(STATUS "MATLAB_FOUNDATION_LIB=${MATLAB_FOUNDATION_LIB}")
	set(MATLAB_LIBRARY_DIR 
		${MATLAB_DIR_SEARCH}/extern/bin/${SUFFIX_FOR_LIBRARY_PATH}
		${MATLAB_DIR_SEARCH}/bin/${SUFFIX_FOR_LIBRARY_PATH}
	)
else()
	message(FATAL_ERROR "Foundation libraries not found (e.g. ${MATLAB_DIR_SEARCH}/extern/bin/${SUFFIX_FOR_LIBRARY_PATH}/libMatlabDataArray${CMAKE_SHARED_LIBRARY_SUFFIX})")
endif()	    

message(STATUS "MATLAB_INCLUDE_DIRS=${MATLAB_INCLUDE_DIRS}")
message(STATUS "MATLAB_LIBRARY_DIR=${MATLAB_LIBRARY_DIR}")
message(STATUS "MATLAB_FOUNDATION_LIB=${MATLAB_FOUNDATION_LIB}")

set (MATLAB_FOUND True)
