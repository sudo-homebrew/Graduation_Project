#ifndef BWI_PLANNING_COMMON__VISIBILITY_CONTROL_H_
#define BWI_PLANNING_COMMON__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define BWI_PLANNING_COMMON_EXPORT __attribute__ ((dllexport))
    #define BWI_PLANNING_COMMON_IMPORT __attribute__ ((dllimport))
  #else
    #define BWI_PLANNING_COMMON_EXPORT __declspec(dllexport)
    #define BWI_PLANNING_COMMON_IMPORT __declspec(dllimport)
  #endif
  #ifdef BWI_PLANNING_COMMON_BUILDING_LIBRARY
    #define BWI_PLANNING_COMMON_PUBLIC BWI_PLANNING_COMMON_EXPORT
  #else
    #define BWI_PLANNING_COMMON_PUBLIC BWI_PLANNING_COMMON_IMPORT
  #endif
  #define BWI_PLANNING_COMMON_PUBLIC_TYPE BWI_PLANNING_COMMON_PUBLIC
  #define BWI_PLANNING_COMMON_LOCAL
#else
  #define BWI_PLANNING_COMMON_EXPORT __attribute__ ((visibility("default")))
  #define BWI_PLANNING_COMMON_IMPORT
  #if __GNUC__ >= 4
    #define BWI_PLANNING_COMMON_PUBLIC __attribute__ ((visibility("default")))
    #define BWI_PLANNING_COMMON_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define BWI_PLANNING_COMMON_PUBLIC
    #define BWI_PLANNING_COMMON_LOCAL
  #endif
  #define BWI_PLANNING_COMMON_PUBLIC_TYPE
#endif
#endif  // BWI_PLANNING_COMMON__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 17:58:34
// Copyright 2019-2020 The MathWorks, Inc.
