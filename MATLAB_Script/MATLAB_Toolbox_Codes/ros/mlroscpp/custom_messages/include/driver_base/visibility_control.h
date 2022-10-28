#ifndef DRIVER_BASE__VISIBILITY_CONTROL_H_
#define DRIVER_BASE__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DRIVER_BASE_EXPORT __attribute__ ((dllexport))
    #define DRIVER_BASE_IMPORT __attribute__ ((dllimport))
  #else
    #define DRIVER_BASE_EXPORT __declspec(dllexport)
    #define DRIVER_BASE_IMPORT __declspec(dllimport)
  #endif
  #ifdef DRIVER_BASE_BUILDING_LIBRARY
    #define DRIVER_BASE_PUBLIC DRIVER_BASE_EXPORT
  #else
    #define DRIVER_BASE_PUBLIC DRIVER_BASE_IMPORT
  #endif
  #define DRIVER_BASE_PUBLIC_TYPE DRIVER_BASE_PUBLIC
  #define DRIVER_BASE_LOCAL
#else
  #define DRIVER_BASE_EXPORT __attribute__ ((visibility("default")))
  #define DRIVER_BASE_IMPORT
  #if __GNUC__ >= 4
    #define DRIVER_BASE_PUBLIC __attribute__ ((visibility("default")))
    #define DRIVER_BASE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DRIVER_BASE_PUBLIC
    #define DRIVER_BASE_LOCAL
  #endif
  #define DRIVER_BASE_PUBLIC_TYPE
#endif
#endif  // DRIVER_BASE__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 17:59:11
// Copyright 2019-2020 The MathWorks, Inc.
