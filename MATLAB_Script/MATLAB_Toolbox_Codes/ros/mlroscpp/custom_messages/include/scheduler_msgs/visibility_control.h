#ifndef SCHEDULER_MSGS__VISIBILITY_CONTROL_H_
#define SCHEDULER_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SCHEDULER_MSGS_EXPORT __attribute__ ((dllexport))
    #define SCHEDULER_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define SCHEDULER_MSGS_EXPORT __declspec(dllexport)
    #define SCHEDULER_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef SCHEDULER_MSGS_BUILDING_LIBRARY
    #define SCHEDULER_MSGS_PUBLIC SCHEDULER_MSGS_EXPORT
  #else
    #define SCHEDULER_MSGS_PUBLIC SCHEDULER_MSGS_IMPORT
  #endif
  #define SCHEDULER_MSGS_PUBLIC_TYPE SCHEDULER_MSGS_PUBLIC
  #define SCHEDULER_MSGS_LOCAL
#else
  #define SCHEDULER_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define SCHEDULER_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define SCHEDULER_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define SCHEDULER_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SCHEDULER_MSGS_PUBLIC
    #define SCHEDULER_MSGS_LOCAL
  #endif
  #define SCHEDULER_MSGS_PUBLIC_TYPE
#endif
#endif  // SCHEDULER_MSGS__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 14:36:21
// Copyright 2019-2020 The MathWorks, Inc.
