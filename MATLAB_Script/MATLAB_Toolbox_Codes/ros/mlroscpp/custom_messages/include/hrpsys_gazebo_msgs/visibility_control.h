#ifndef HRPSYS_GAZEBO_MSGS__VISIBILITY_CONTROL_H_
#define HRPSYS_GAZEBO_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define HRPSYS_GAZEBO_MSGS_EXPORT __attribute__ ((dllexport))
    #define HRPSYS_GAZEBO_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define HRPSYS_GAZEBO_MSGS_EXPORT __declspec(dllexport)
    #define HRPSYS_GAZEBO_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef HRPSYS_GAZEBO_MSGS_BUILDING_LIBRARY
    #define HRPSYS_GAZEBO_MSGS_PUBLIC HRPSYS_GAZEBO_MSGS_EXPORT
  #else
    #define HRPSYS_GAZEBO_MSGS_PUBLIC HRPSYS_GAZEBO_MSGS_IMPORT
  #endif
  #define HRPSYS_GAZEBO_MSGS_PUBLIC_TYPE HRPSYS_GAZEBO_MSGS_PUBLIC
  #define HRPSYS_GAZEBO_MSGS_LOCAL
#else
  #define HRPSYS_GAZEBO_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define HRPSYS_GAZEBO_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define HRPSYS_GAZEBO_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define HRPSYS_GAZEBO_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define HRPSYS_GAZEBO_MSGS_PUBLIC
    #define HRPSYS_GAZEBO_MSGS_LOCAL
  #endif
  #define HRPSYS_GAZEBO_MSGS_PUBLIC_TYPE
#endif
#endif  // HRPSYS_GAZEBO_MSGS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 17:59:55
// Copyright 2019-2020 The MathWorks, Inc.
