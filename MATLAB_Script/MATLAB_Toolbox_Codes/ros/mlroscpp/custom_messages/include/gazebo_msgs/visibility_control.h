#ifndef GAZEBO_MSGS__VISIBILITY_CONTROL_H_
#define GAZEBO_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GAZEBO_MSGS_EXPORT __attribute__ ((dllexport))
    #define GAZEBO_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define GAZEBO_MSGS_EXPORT __declspec(dllexport)
    #define GAZEBO_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef GAZEBO_MSGS_BUILDING_LIBRARY
    #define GAZEBO_MSGS_PUBLIC GAZEBO_MSGS_EXPORT
  #else
    #define GAZEBO_MSGS_PUBLIC GAZEBO_MSGS_IMPORT
  #endif
  #define GAZEBO_MSGS_PUBLIC_TYPE GAZEBO_MSGS_PUBLIC
  #define GAZEBO_MSGS_LOCAL
#else
  #define GAZEBO_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define GAZEBO_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define GAZEBO_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define GAZEBO_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GAZEBO_MSGS_PUBLIC
    #define GAZEBO_MSGS_LOCAL
  #endif
  #define GAZEBO_MSGS_PUBLIC_TYPE
#endif
#endif  // GAZEBO_MSGS__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 13:24:09
// Copyright 2019-2020 The MathWorks, Inc.
