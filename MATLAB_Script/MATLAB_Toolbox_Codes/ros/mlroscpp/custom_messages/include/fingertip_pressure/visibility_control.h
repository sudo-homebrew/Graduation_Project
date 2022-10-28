#ifndef FINGERTIP_PRESSURE__VISIBILITY_CONTROL_H_
#define FINGERTIP_PRESSURE__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define FINGERTIP_PRESSURE_EXPORT __attribute__ ((dllexport))
    #define FINGERTIP_PRESSURE_IMPORT __attribute__ ((dllimport))
  #else
    #define FINGERTIP_PRESSURE_EXPORT __declspec(dllexport)
    #define FINGERTIP_PRESSURE_IMPORT __declspec(dllimport)
  #endif
  #ifdef FINGERTIP_PRESSURE_BUILDING_LIBRARY
    #define FINGERTIP_PRESSURE_PUBLIC FINGERTIP_PRESSURE_EXPORT
  #else
    #define FINGERTIP_PRESSURE_PUBLIC FINGERTIP_PRESSURE_IMPORT
  #endif
  #define FINGERTIP_PRESSURE_PUBLIC_TYPE FINGERTIP_PRESSURE_PUBLIC
  #define FINGERTIP_PRESSURE_LOCAL
#else
  #define FINGERTIP_PRESSURE_EXPORT __attribute__ ((visibility("default")))
  #define FINGERTIP_PRESSURE_IMPORT
  #if __GNUC__ >= 4
    #define FINGERTIP_PRESSURE_PUBLIC __attribute__ ((visibility("default")))
    #define FINGERTIP_PRESSURE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define FINGERTIP_PRESSURE_PUBLIC
    #define FINGERTIP_PRESSURE_LOCAL
  #endif
  #define FINGERTIP_PRESSURE_PUBLIC_TYPE
#endif
#endif  // FINGERTIP_PRESSURE__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 17:59:25
// Copyright 2019-2020 The MathWorks, Inc.
