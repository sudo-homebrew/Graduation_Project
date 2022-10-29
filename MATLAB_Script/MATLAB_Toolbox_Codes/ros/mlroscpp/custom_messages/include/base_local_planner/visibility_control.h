#ifndef BASE_LOCAL_PLANNER__VISIBILITY_CONTROL_H_
#define BASE_LOCAL_PLANNER__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define BASE_LOCAL_PLANNER_EXPORT __attribute__ ((dllexport))
    #define BASE_LOCAL_PLANNER_IMPORT __attribute__ ((dllimport))
  #else
    #define BASE_LOCAL_PLANNER_EXPORT __declspec(dllexport)
    #define BASE_LOCAL_PLANNER_IMPORT __declspec(dllimport)
  #endif
  #ifdef BASE_LOCAL_PLANNER_BUILDING_LIBRARY
    #define BASE_LOCAL_PLANNER_PUBLIC BASE_LOCAL_PLANNER_EXPORT
  #else
    #define BASE_LOCAL_PLANNER_PUBLIC BASE_LOCAL_PLANNER_IMPORT
  #endif
  #define BASE_LOCAL_PLANNER_PUBLIC_TYPE BASE_LOCAL_PLANNER_PUBLIC
  #define BASE_LOCAL_PLANNER_LOCAL
#else
  #define BASE_LOCAL_PLANNER_EXPORT __attribute__ ((visibility("default")))
  #define BASE_LOCAL_PLANNER_IMPORT
  #if __GNUC__ >= 4
    #define BASE_LOCAL_PLANNER_PUBLIC __attribute__ ((visibility("default")))
    #define BASE_LOCAL_PLANNER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define BASE_LOCAL_PLANNER_PUBLIC
    #define BASE_LOCAL_PLANNER_LOCAL
  #endif
  #define BASE_LOCAL_PLANNER_PUBLIC_TYPE
#endif
#endif  // BASE_LOCAL_PLANNER__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 17:58:19
// Copyright 2019-2020 The MathWorks, Inc.
