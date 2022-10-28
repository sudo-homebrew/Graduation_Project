#ifndef SR_ROBOT_MSGS__VISIBILITY_CONTROL_H_
#define SR_ROBOT_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SR_ROBOT_MSGS_EXPORT __attribute__ ((dllexport))
    #define SR_ROBOT_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define SR_ROBOT_MSGS_EXPORT __declspec(dllexport)
    #define SR_ROBOT_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef SR_ROBOT_MSGS_BUILDING_LIBRARY
    #define SR_ROBOT_MSGS_PUBLIC SR_ROBOT_MSGS_EXPORT
  #else
    #define SR_ROBOT_MSGS_PUBLIC SR_ROBOT_MSGS_IMPORT
  #endif
  #define SR_ROBOT_MSGS_PUBLIC_TYPE SR_ROBOT_MSGS_PUBLIC
  #define SR_ROBOT_MSGS_LOCAL
#else
  #define SR_ROBOT_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define SR_ROBOT_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define SR_ROBOT_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define SR_ROBOT_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SR_ROBOT_MSGS_PUBLIC
    #define SR_ROBOT_MSGS_LOCAL
  #endif
  #define SR_ROBOT_MSGS_PUBLIC_TYPE
#endif
#endif  // SR_ROBOT_MSGS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:02:50
// Copyright 2019-2020 The MathWorks, Inc.
