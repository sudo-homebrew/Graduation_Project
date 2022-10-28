#ifndef MOVE_BASE_MSGS__VISIBILITY_CONTROL_H_
#define MOVE_BASE_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MOVE_BASE_MSGS_EXPORT __attribute__ ((dllexport))
    #define MOVE_BASE_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define MOVE_BASE_MSGS_EXPORT __declspec(dllexport)
    #define MOVE_BASE_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef MOVE_BASE_MSGS_BUILDING_LIBRARY
    #define MOVE_BASE_MSGS_PUBLIC MOVE_BASE_MSGS_EXPORT
  #else
    #define MOVE_BASE_MSGS_PUBLIC MOVE_BASE_MSGS_IMPORT
  #endif
  #define MOVE_BASE_MSGS_PUBLIC_TYPE MOVE_BASE_MSGS_PUBLIC
  #define MOVE_BASE_MSGS_LOCAL
#else
  #define MOVE_BASE_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define MOVE_BASE_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define MOVE_BASE_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define MOVE_BASE_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MOVE_BASE_MSGS_PUBLIC
    #define MOVE_BASE_MSGS_LOCAL
  #endif
  #define MOVE_BASE_MSGS_PUBLIC_TYPE
#endif
#endif  // MOVE_BASE_MSGS__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 14:01:48
// Copyright 2019-2020 The MathWorks, Inc.
