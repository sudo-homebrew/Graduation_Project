#ifndef ROSAPI__VISIBILITY_CONTROL_H_
#define ROSAPI__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSAPI_EXPORT __attribute__ ((dllexport))
    #define ROSAPI_IMPORT __attribute__ ((dllimport))
  #else
    #define ROSAPI_EXPORT __declspec(dllexport)
    #define ROSAPI_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROSAPI_BUILDING_LIBRARY
    #define ROSAPI_PUBLIC ROSAPI_EXPORT
  #else
    #define ROSAPI_PUBLIC ROSAPI_IMPORT
  #endif
  #define ROSAPI_PUBLIC_TYPE ROSAPI_PUBLIC
  #define ROSAPI_LOCAL
#else
  #define ROSAPI_EXPORT __attribute__ ((visibility("default")))
  #define ROSAPI_IMPORT
  #if __GNUC__ >= 4
    #define ROSAPI_PUBLIC __attribute__ ((visibility("default")))
    #define ROSAPI_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROSAPI_PUBLIC
    #define ROSAPI_LOCAL
  #endif
  #define ROSAPI_PUBLIC_TYPE
#endif
#endif  // ROSAPI__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:02:10
// Copyright 2019-2020 The MathWorks, Inc.
