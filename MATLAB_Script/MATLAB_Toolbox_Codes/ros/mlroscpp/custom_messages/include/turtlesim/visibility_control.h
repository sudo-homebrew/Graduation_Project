#ifndef TURTLESIM__VISIBILITY_CONTROL_H_
#define TURTLESIM__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TURTLESIM_EXPORT __attribute__ ((dllexport))
    #define TURTLESIM_IMPORT __attribute__ ((dllimport))
  #else
    #define TURTLESIM_EXPORT __declspec(dllexport)
    #define TURTLESIM_IMPORT __declspec(dllimport)
  #endif
  #ifdef TURTLESIM_BUILDING_LIBRARY
    #define TURTLESIM_PUBLIC TURTLESIM_EXPORT
  #else
    #define TURTLESIM_PUBLIC TURTLESIM_IMPORT
  #endif
  #define TURTLESIM_PUBLIC_TYPE TURTLESIM_PUBLIC
  #define TURTLESIM_LOCAL
#else
  #define TURTLESIM_EXPORT __attribute__ ((visibility("default")))
  #define TURTLESIM_IMPORT
  #if __GNUC__ >= 4
    #define TURTLESIM_PUBLIC __attribute__ ((visibility("default")))
    #define TURTLESIM_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TURTLESIM_PUBLIC
    #define TURTLESIM_LOCAL
  #endif
  #define TURTLESIM_PUBLIC_TYPE
#endif
#endif  // TURTLESIM__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:03:17
// Copyright 2019-2020 The MathWorks, Inc.
