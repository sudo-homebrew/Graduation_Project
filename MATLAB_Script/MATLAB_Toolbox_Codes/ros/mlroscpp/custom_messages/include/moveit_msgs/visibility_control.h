#ifndef MOVEIT_MSGS__VISIBILITY_CONTROL_H_
#define MOVEIT_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MOVEIT_MSGS_EXPORT __attribute__ ((dllexport))
    #define MOVEIT_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define MOVEIT_MSGS_EXPORT __declspec(dllexport)
    #define MOVEIT_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef MOVEIT_MSGS_BUILDING_LIBRARY
    #define MOVEIT_MSGS_PUBLIC MOVEIT_MSGS_EXPORT
  #else
    #define MOVEIT_MSGS_PUBLIC MOVEIT_MSGS_IMPORT
  #endif
  #define MOVEIT_MSGS_PUBLIC_TYPE MOVEIT_MSGS_PUBLIC
  #define MOVEIT_MSGS_LOCAL
#else
  #define MOVEIT_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define MOVEIT_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define MOVEIT_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define MOVEIT_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MOVEIT_MSGS_PUBLIC
    #define MOVEIT_MSGS_LOCAL
  #endif
  #define MOVEIT_MSGS_PUBLIC_TYPE
#endif
#endif  // MOVEIT_MSGS__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 14:26:17
// Copyright 2019-2020 The MathWorks, Inc.
