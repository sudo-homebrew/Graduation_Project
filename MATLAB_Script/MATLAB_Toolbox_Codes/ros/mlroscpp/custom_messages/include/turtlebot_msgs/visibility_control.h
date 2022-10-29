#ifndef TURTLEBOT_MSGS__VISIBILITY_CONTROL_H_
#define TURTLEBOT_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TURTLEBOT_MSGS_EXPORT __attribute__ ((dllexport))
    #define TURTLEBOT_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define TURTLEBOT_MSGS_EXPORT __declspec(dllexport)
    #define TURTLEBOT_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef TURTLEBOT_MSGS_BUILDING_LIBRARY
    #define TURTLEBOT_MSGS_PUBLIC TURTLEBOT_MSGS_EXPORT
  #else
    #define TURTLEBOT_MSGS_PUBLIC TURTLEBOT_MSGS_IMPORT
  #endif
  #define TURTLEBOT_MSGS_PUBLIC_TYPE TURTLEBOT_MSGS_PUBLIC
  #define TURTLEBOT_MSGS_LOCAL
#else
  #define TURTLEBOT_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define TURTLEBOT_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define TURTLEBOT_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define TURTLEBOT_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TURTLEBOT_MSGS_PUBLIC
    #define TURTLEBOT_MSGS_LOCAL
  #endif
  #define TURTLEBOT_MSGS_PUBLIC_TYPE
#endif
#endif  // TURTLEBOT_MSGS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:03:15
// Copyright 2019-2020 The MathWorks, Inc.
