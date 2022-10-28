#ifndef GRASPING_MSGS__VISIBILITY_CONTROL_H_
#define GRASPING_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GRASPING_MSGS_EXPORT __attribute__ ((dllexport))
    #define GRASPING_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define GRASPING_MSGS_EXPORT __declspec(dllexport)
    #define GRASPING_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef GRASPING_MSGS_BUILDING_LIBRARY
    #define GRASPING_MSGS_PUBLIC GRASPING_MSGS_EXPORT
  #else
    #define GRASPING_MSGS_PUBLIC GRASPING_MSGS_IMPORT
  #endif
  #define GRASPING_MSGS_PUBLIC_TYPE GRASPING_MSGS_PUBLIC
  #define GRASPING_MSGS_LOCAL
#else
  #define GRASPING_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define GRASPING_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define GRASPING_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define GRASPING_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GRASPING_MSGS_PUBLIC
    #define GRASPING_MSGS_LOCAL
  #endif
  #define GRASPING_MSGS_PUBLIC_TYPE
#endif
#endif  // GRASPING_MSGS__VISIBILITY_CONTROL_H_
// Generated 12-Apr-2020 11:46:31
// Copyright 2019-2020 The MathWorks, Inc.
