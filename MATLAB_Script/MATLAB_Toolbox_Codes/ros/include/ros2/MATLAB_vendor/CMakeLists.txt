#CMakeLists.txt
#Copyright 2018-2021 The MathWorks, Inc.
#Copy this and other files to custom message definition
#This will build publisher and subscriber files 
cmake_minimum_required(VERSION 3.9)

project(MATLAB_vendor)

set(PACKAGE_VERSION "1.0.0")

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
find_package(MATLAB)

if (NOT MATLAB_FOUND)
	install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/MATLAB_install/ DESTINATION ${CMAKE_INSTALL_PREFIX})
endif()

configure_file(MATLAB_vendorConfig.cmake.in "${PROJECT_BINARY_DIR}/MATLAB_vendorConfig.cmake" @ONLY)
configure_file(MATLAB_vendorConfig-version.cmake.in "${PROJECT_BINARY_DIR}/MATLAB_vendorConfig-version.cmake" @ONLY)

install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})

install(FILES
  "${PROJECT_BINARY_DIR}/MATLAB_vendorConfig.cmake"
  "${PROJECT_BINARY_DIR}/MATLAB_vendorConfig-version.cmake"
  DESTINATION share/${PROJECT_NAME}/cmake)