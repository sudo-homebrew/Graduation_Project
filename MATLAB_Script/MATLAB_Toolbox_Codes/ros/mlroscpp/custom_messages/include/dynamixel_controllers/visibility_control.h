#ifndef DYNAMIXEL_CONTROLLERS__VISIBILITY_CONTROL_H_
#define DYNAMIXEL_CONTROLLERS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DYNAMIXEL_CONTROLLERS_EXPORT __attribute__ ((dllexport))
    #define DYNAMIXEL_CONTROLLERS_IMPORT __attribute__ ((dllimport))
  #else
    #define DYNAMIXEL_CONTROLLERS_EXPORT __declspec(dllexport)
    #define DYNAMIXEL_CONTROLLERS_IMPORT __declspec(dllimport)
  #endif
  #ifdef DYNAMIXEL_CONTROLLERS_BUILDING_LIBRARY
    #define DYNAMIXEL_CONTROLLERS_PUBLIC DYNAMIXEL_CONTROLLERS_EXPORT
  #else
    #define DYNAMIXEL_CONTROLLERS_PUBLIC DYNAMIXEL_CONTROLLERS_IMPORT
  #endif
  #define DYNAMIXEL_CONTROLLERS_PUBLIC_TYPE DYNAMIXEL_CONTROLLERS_PUBLIC
  #define DYNAMIXEL_CONTROLLERS_LOCAL
#else
  #define DYNAMIXEL_CONTROLLERS_EXPORT __attribute__ ((visibility("default")))
  #define DYNAMIXEL_CONTROLLERS_IMPORT
  #if __GNUC__ >= 4
    #define DYNAMIXEL_CONTROLLERS_PUBLIC __attribute__ ((visibility("default")))
    #define DYNAMIXEL_CONTROLLERS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DYNAMIXEL_CONTROLLERS_PUBLIC
    #define DYNAMIXEL_CONTROLLERS_LOCAL
  #endif
  #define DYNAMIXEL_CONTROLLERS_PUBLIC_TYPE
#endif
#endif  // DYNAMIXEL_CONTROLLERS__VISIBILITY_CONTROL_H_
// Generated 06-Apr-2020 15:38:54
// Copyright 2019-2020 The MathWorks, Inc.
