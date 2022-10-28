#ifndef DYNAMIXEL_MSGS__VISIBILITY_CONTROL_H_
#define DYNAMIXEL_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DYNAMIXEL_MSGS_EXPORT __attribute__ ((dllexport))
    #define DYNAMIXEL_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define DYNAMIXEL_MSGS_EXPORT __declspec(dllexport)
    #define DYNAMIXEL_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef DYNAMIXEL_MSGS_BUILDING_LIBRARY
    #define DYNAMIXEL_MSGS_PUBLIC DYNAMIXEL_MSGS_EXPORT
  #else
    #define DYNAMIXEL_MSGS_PUBLIC DYNAMIXEL_MSGS_IMPORT
  #endif
  #define DYNAMIXEL_MSGS_PUBLIC_TYPE DYNAMIXEL_MSGS_PUBLIC
  #define DYNAMIXEL_MSGS_LOCAL
#else
  #define DYNAMIXEL_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define DYNAMIXEL_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define DYNAMIXEL_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define DYNAMIXEL_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DYNAMIXEL_MSGS_PUBLIC
    #define DYNAMIXEL_MSGS_LOCAL
  #endif
  #define DYNAMIXEL_MSGS_PUBLIC_TYPE
#endif
#endif  // DYNAMIXEL_MSGS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 17:59:15
// Copyright 2019-2020 The MathWorks, Inc.
