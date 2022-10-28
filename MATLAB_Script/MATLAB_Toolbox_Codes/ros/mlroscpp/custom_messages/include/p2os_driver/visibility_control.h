#ifndef P2OS_DRIVER__VISIBILITY_CONTROL_H_
#define P2OS_DRIVER__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define P2OS_DRIVER_EXPORT __attribute__ ((dllexport))
    #define P2OS_DRIVER_IMPORT __attribute__ ((dllimport))
  #else
    #define P2OS_DRIVER_EXPORT __declspec(dllexport)
    #define P2OS_DRIVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef P2OS_DRIVER_BUILDING_LIBRARY
    #define P2OS_DRIVER_PUBLIC P2OS_DRIVER_EXPORT
  #else
    #define P2OS_DRIVER_PUBLIC P2OS_DRIVER_IMPORT
  #endif
  #define P2OS_DRIVER_PUBLIC_TYPE P2OS_DRIVER_PUBLIC
  #define P2OS_DRIVER_LOCAL
#else
  #define P2OS_DRIVER_EXPORT __attribute__ ((visibility("default")))
  #define P2OS_DRIVER_IMPORT
  #if __GNUC__ >= 4
    #define P2OS_DRIVER_PUBLIC __attribute__ ((visibility("default")))
    #define P2OS_DRIVER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define P2OS_DRIVER_PUBLIC
    #define P2OS_DRIVER_LOCAL
  #endif
  #define P2OS_DRIVER_PUBLIC_TYPE
#endif
#endif  // P2OS_DRIVER__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:01:20
// Copyright 2019-2020 The MathWorks, Inc.
