#ifndef TURTLEBOT_ACTIONS__VISIBILITY_CONTROL_H_
#define TURTLEBOT_ACTIONS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TURTLEBOT_ACTIONS_EXPORT __attribute__ ((dllexport))
    #define TURTLEBOT_ACTIONS_IMPORT __attribute__ ((dllimport))
  #else
    #define TURTLEBOT_ACTIONS_EXPORT __declspec(dllexport)
    #define TURTLEBOT_ACTIONS_IMPORT __declspec(dllimport)
  #endif
  #ifdef TURTLEBOT_ACTIONS_BUILDING_LIBRARY
    #define TURTLEBOT_ACTIONS_PUBLIC TURTLEBOT_ACTIONS_EXPORT
  #else
    #define TURTLEBOT_ACTIONS_PUBLIC TURTLEBOT_ACTIONS_IMPORT
  #endif
  #define TURTLEBOT_ACTIONS_PUBLIC_TYPE TURTLEBOT_ACTIONS_PUBLIC
  #define TURTLEBOT_ACTIONS_LOCAL
#else
  #define TURTLEBOT_ACTIONS_EXPORT __attribute__ ((visibility("default")))
  #define TURTLEBOT_ACTIONS_IMPORT
  #if __GNUC__ >= 4
    #define TURTLEBOT_ACTIONS_PUBLIC __attribute__ ((visibility("default")))
    #define TURTLEBOT_ACTIONS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TURTLEBOT_ACTIONS_PUBLIC
    #define TURTLEBOT_ACTIONS_LOCAL
  #endif
  #define TURTLEBOT_ACTIONS_PUBLIC_TYPE
#endif
#endif  // TURTLEBOT_ACTIONS__VISIBILITY_CONTROL_H_
// Generated 11-Apr-2020 16:38:23
// Copyright 2019-2020 The MathWorks, Inc.
