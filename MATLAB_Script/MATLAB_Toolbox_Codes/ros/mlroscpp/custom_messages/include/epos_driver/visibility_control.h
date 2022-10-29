#ifndef EPOS_DRIVER__VISIBILITY_CONTROL_H_
#define EPOS_DRIVER__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define EPOS_DRIVER_EXPORT __attribute__ ((dllexport))
    #define EPOS_DRIVER_IMPORT __attribute__ ((dllimport))
  #else
    #define EPOS_DRIVER_EXPORT __declspec(dllexport)
    #define EPOS_DRIVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef EPOS_DRIVER_BUILDING_LIBRARY
    #define EPOS_DRIVER_PUBLIC EPOS_DRIVER_EXPORT
  #else
    #define EPOS_DRIVER_PUBLIC EPOS_DRIVER_IMPORT
  #endif
  #define EPOS_DRIVER_PUBLIC_TYPE EPOS_DRIVER_PUBLIC
  #define EPOS_DRIVER_LOCAL
#else
  #define EPOS_DRIVER_EXPORT __attribute__ ((visibility("default")))
  #define EPOS_DRIVER_IMPORT
  #if __GNUC__ >= 4
    #define EPOS_DRIVER_PUBLIC __attribute__ ((visibility("default")))
    #define EPOS_DRIVER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define EPOS_DRIVER_PUBLIC
    #define EPOS_DRIVER_LOCAL
  #endif
  #define EPOS_DRIVER_PUBLIC_TYPE
#endif
#endif  // EPOS_DRIVER__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 17:59:17
// Copyright 2019-2020 The MathWorks, Inc.
