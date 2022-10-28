#ifndef UNDERWATER_SENSOR_MSGS__VISIBILITY_CONTROL_H_
#define UNDERWATER_SENSOR_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define UNDERWATER_SENSOR_MSGS_EXPORT __attribute__ ((dllexport))
    #define UNDERWATER_SENSOR_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define UNDERWATER_SENSOR_MSGS_EXPORT __declspec(dllexport)
    #define UNDERWATER_SENSOR_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef UNDERWATER_SENSOR_MSGS_BUILDING_LIBRARY
    #define UNDERWATER_SENSOR_MSGS_PUBLIC UNDERWATER_SENSOR_MSGS_EXPORT
  #else
    #define UNDERWATER_SENSOR_MSGS_PUBLIC UNDERWATER_SENSOR_MSGS_IMPORT
  #endif
  #define UNDERWATER_SENSOR_MSGS_PUBLIC_TYPE UNDERWATER_SENSOR_MSGS_PUBLIC
  #define UNDERWATER_SENSOR_MSGS_LOCAL
#else
  #define UNDERWATER_SENSOR_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define UNDERWATER_SENSOR_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define UNDERWATER_SENSOR_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define UNDERWATER_SENSOR_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define UNDERWATER_SENSOR_MSGS_PUBLIC
    #define UNDERWATER_SENSOR_MSGS_LOCAL
  #endif
  #define UNDERWATER_SENSOR_MSGS_PUBLIC_TYPE
#endif
#endif  // UNDERWATER_SENSOR_MSGS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:03:19
// Copyright 2019-2020 The MathWorks, Inc.
