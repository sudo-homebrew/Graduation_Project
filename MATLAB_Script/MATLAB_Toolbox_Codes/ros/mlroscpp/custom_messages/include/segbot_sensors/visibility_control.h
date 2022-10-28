#ifndef SEGBOT_SENSORS__VISIBILITY_CONTROL_H_
#define SEGBOT_SENSORS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SEGBOT_SENSORS_EXPORT __attribute__ ((dllexport))
    #define SEGBOT_SENSORS_IMPORT __attribute__ ((dllimport))
  #else
    #define SEGBOT_SENSORS_EXPORT __declspec(dllexport)
    #define SEGBOT_SENSORS_IMPORT __declspec(dllimport)
  #endif
  #ifdef SEGBOT_SENSORS_BUILDING_LIBRARY
    #define SEGBOT_SENSORS_PUBLIC SEGBOT_SENSORS_EXPORT
  #else
    #define SEGBOT_SENSORS_PUBLIC SEGBOT_SENSORS_IMPORT
  #endif
  #define SEGBOT_SENSORS_PUBLIC_TYPE SEGBOT_SENSORS_PUBLIC
  #define SEGBOT_SENSORS_LOCAL
#else
  #define SEGBOT_SENSORS_EXPORT __attribute__ ((visibility("default")))
  #define SEGBOT_SENSORS_IMPORT
  #if __GNUC__ >= 4
    #define SEGBOT_SENSORS_PUBLIC __attribute__ ((visibility("default")))
    #define SEGBOT_SENSORS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SEGBOT_SENSORS_PUBLIC
    #define SEGBOT_SENSORS_LOCAL
  #endif
  #define SEGBOT_SENSORS_PUBLIC_TYPE
#endif
#endif  // SEGBOT_SENSORS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:02:30
// Copyright 2019-2020 The MathWorks, Inc.
