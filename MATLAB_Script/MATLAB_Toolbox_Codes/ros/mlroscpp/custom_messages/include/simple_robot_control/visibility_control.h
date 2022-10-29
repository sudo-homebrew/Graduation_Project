#ifndef SIMPLE_ROBOT_CONTROL__VISIBILITY_CONTROL_H_
#define SIMPLE_ROBOT_CONTROL__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SIMPLE_ROBOT_CONTROL_EXPORT __attribute__ ((dllexport))
    #define SIMPLE_ROBOT_CONTROL_IMPORT __attribute__ ((dllimport))
  #else
    #define SIMPLE_ROBOT_CONTROL_EXPORT __declspec(dllexport)
    #define SIMPLE_ROBOT_CONTROL_IMPORT __declspec(dllimport)
  #endif
  #ifdef SIMPLE_ROBOT_CONTROL_BUILDING_LIBRARY
    #define SIMPLE_ROBOT_CONTROL_PUBLIC SIMPLE_ROBOT_CONTROL_EXPORT
  #else
    #define SIMPLE_ROBOT_CONTROL_PUBLIC SIMPLE_ROBOT_CONTROL_IMPORT
  #endif
  #define SIMPLE_ROBOT_CONTROL_PUBLIC_TYPE SIMPLE_ROBOT_CONTROL_PUBLIC
  #define SIMPLE_ROBOT_CONTROL_LOCAL
#else
  #define SIMPLE_ROBOT_CONTROL_EXPORT __attribute__ ((visibility("default")))
  #define SIMPLE_ROBOT_CONTROL_IMPORT
  #if __GNUC__ >= 4
    #define SIMPLE_ROBOT_CONTROL_PUBLIC __attribute__ ((visibility("default")))
    #define SIMPLE_ROBOT_CONTROL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SIMPLE_ROBOT_CONTROL_PUBLIC
    #define SIMPLE_ROBOT_CONTROL_LOCAL
  #endif
  #define SIMPLE_ROBOT_CONTROL_PUBLIC_TYPE
#endif
#endif  // SIMPLE_ROBOT_CONTROL__VISIBILITY_CONTROL_H_
// Generated 25-Mar-2020 22:24:59
// Copyright 2019-2020 The MathWorks, Inc.
