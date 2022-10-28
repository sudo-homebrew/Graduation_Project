#ifndef STATISTICS_MSGS__VISIBILITY_CONTROL_H_
#define STATISTICS_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define STATISTICS_MSGS_EXPORT __attribute__ ((dllexport))
    #define STATISTICS_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define STATISTICS_MSGS_EXPORT __declspec(dllexport)
    #define STATISTICS_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef STATISTICS_MSGS_BUILDING_LIBRARY
    #define STATISTICS_MSGS_PUBLIC STATISTICS_MSGS_EXPORT
  #else
    #define STATISTICS_MSGS_PUBLIC STATISTICS_MSGS_IMPORT
  #endif
  #define STATISTICS_MSGS_PUBLIC_TYPE STATISTICS_MSGS_PUBLIC
  #define STATISTICS_MSGS_LOCAL
#else
  #define STATISTICS_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define STATISTICS_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define STATISTICS_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define STATISTICS_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define STATISTICS_MSGS_PUBLIC
    #define STATISTICS_MSGS_LOCAL
  #endif
  #define STATISTICS_MSGS_PUBLIC_TYPE
#endif
#endif  // STATISTICS_MSGS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:02:54
// Copyright 2019-2020 The MathWorks, Inc.
