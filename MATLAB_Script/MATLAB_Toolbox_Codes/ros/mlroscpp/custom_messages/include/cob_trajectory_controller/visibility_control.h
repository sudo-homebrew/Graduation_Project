#ifndef COB_TRAJECTORY_CONTROLLER__VISIBILITY_CONTROL_H_
#define COB_TRAJECTORY_CONTROLLER__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define COB_TRAJECTORY_CONTROLLER_EXPORT __attribute__ ((dllexport))
    #define COB_TRAJECTORY_CONTROLLER_IMPORT __attribute__ ((dllimport))
  #else
    #define COB_TRAJECTORY_CONTROLLER_EXPORT __declspec(dllexport)
    #define COB_TRAJECTORY_CONTROLLER_IMPORT __declspec(dllimport)
  #endif
  #ifdef COB_TRAJECTORY_CONTROLLER_BUILDING_LIBRARY
    #define COB_TRAJECTORY_CONTROLLER_PUBLIC COB_TRAJECTORY_CONTROLLER_EXPORT
  #else
    #define COB_TRAJECTORY_CONTROLLER_PUBLIC COB_TRAJECTORY_CONTROLLER_IMPORT
  #endif
  #define COB_TRAJECTORY_CONTROLLER_PUBLIC_TYPE COB_TRAJECTORY_CONTROLLER_PUBLIC
  #define COB_TRAJECTORY_CONTROLLER_LOCAL
#else
  #define COB_TRAJECTORY_CONTROLLER_EXPORT __attribute__ ((visibility("default")))
  #define COB_TRAJECTORY_CONTROLLER_IMPORT
  #if __GNUC__ >= 4
    #define COB_TRAJECTORY_CONTROLLER_PUBLIC __attribute__ ((visibility("default")))
    #define COB_TRAJECTORY_CONTROLLER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define COB_TRAJECTORY_CONTROLLER_PUBLIC
    #define COB_TRAJECTORY_CONTROLLER_LOCAL
  #endif
  #define COB_TRAJECTORY_CONTROLLER_PUBLIC_TYPE
#endif
#endif  // COB_TRAJECTORY_CONTROLLER__VISIBILITY_CONTROL_H_
// Generated 25-Mar-2020 19:29:30
// Copyright 2019-2020 The MathWorks, Inc.