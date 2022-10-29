#ifndef INTERVAL_INTERSECTION__VISIBILITY_CONTROL_H_
#define INTERVAL_INTERSECTION__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define INTERVAL_INTERSECTION_EXPORT __attribute__ ((dllexport))
    #define INTERVAL_INTERSECTION_IMPORT __attribute__ ((dllimport))
  #else
    #define INTERVAL_INTERSECTION_EXPORT __declspec(dllexport)
    #define INTERVAL_INTERSECTION_IMPORT __declspec(dllimport)
  #endif
  #ifdef INTERVAL_INTERSECTION_BUILDING_LIBRARY
    #define INTERVAL_INTERSECTION_PUBLIC INTERVAL_INTERSECTION_EXPORT
  #else
    #define INTERVAL_INTERSECTION_PUBLIC INTERVAL_INTERSECTION_IMPORT
  #endif
  #define INTERVAL_INTERSECTION_PUBLIC_TYPE INTERVAL_INTERSECTION_PUBLIC
  #define INTERVAL_INTERSECTION_LOCAL
#else
  #define INTERVAL_INTERSECTION_EXPORT __attribute__ ((visibility("default")))
  #define INTERVAL_INTERSECTION_IMPORT
  #if __GNUC__ >= 4
    #define INTERVAL_INTERSECTION_PUBLIC __attribute__ ((visibility("default")))
    #define INTERVAL_INTERSECTION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define INTERVAL_INTERSECTION_PUBLIC
    #define INTERVAL_INTERSECTION_LOCAL
  #endif
  #define INTERVAL_INTERSECTION_PUBLIC_TYPE
#endif
#endif  // INTERVAL_INTERSECTION__VISIBILITY_CONTROL_H_
// Generated 12-Apr-2020 11:46:34
// Copyright 2019-2020 The MathWorks, Inc.
