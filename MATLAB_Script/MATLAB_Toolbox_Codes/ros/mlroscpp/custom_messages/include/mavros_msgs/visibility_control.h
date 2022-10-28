#ifndef MAVROS_MSGS__VISIBILITY_CONTROL_H_
#define MAVROS_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MAVROS_MSGS_EXPORT __attribute__ ((dllexport))
    #define MAVROS_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define MAVROS_MSGS_EXPORT __declspec(dllexport)
    #define MAVROS_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef MAVROS_MSGS_BUILDING_LIBRARY
    #define MAVROS_MSGS_PUBLIC MAVROS_MSGS_EXPORT
  #else
    #define MAVROS_MSGS_PUBLIC MAVROS_MSGS_IMPORT
  #endif
  #define MAVROS_MSGS_PUBLIC_TYPE MAVROS_MSGS_PUBLIC
  #define MAVROS_MSGS_LOCAL
#else
  #define MAVROS_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define MAVROS_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define MAVROS_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define MAVROS_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MAVROS_MSGS_PUBLIC
    #define MAVROS_MSGS_LOCAL
  #endif
  #define MAVROS_MSGS_PUBLIC_TYPE
#endif
#endif  // MAVROS_MSGS__VISIBILITY_CONTROL_H_
// Generated 13-Jun-2021 18:12:04
// Copyright 2019-2020 The MathWorks, Inc.
