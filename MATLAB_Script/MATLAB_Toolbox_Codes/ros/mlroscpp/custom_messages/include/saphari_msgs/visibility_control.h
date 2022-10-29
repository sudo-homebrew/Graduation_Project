#ifndef SAPHARI_MSGS__VISIBILITY_CONTROL_H_
#define SAPHARI_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SAPHARI_MSGS_EXPORT __attribute__ ((dllexport))
    #define SAPHARI_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define SAPHARI_MSGS_EXPORT __declspec(dllexport)
    #define SAPHARI_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef SAPHARI_MSGS_BUILDING_LIBRARY
    #define SAPHARI_MSGS_PUBLIC SAPHARI_MSGS_EXPORT
  #else
    #define SAPHARI_MSGS_PUBLIC SAPHARI_MSGS_IMPORT
  #endif
  #define SAPHARI_MSGS_PUBLIC_TYPE SAPHARI_MSGS_PUBLIC
  #define SAPHARI_MSGS_LOCAL
#else
  #define SAPHARI_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define SAPHARI_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define SAPHARI_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define SAPHARI_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SAPHARI_MSGS_PUBLIC
    #define SAPHARI_MSGS_LOCAL
  #endif
  #define SAPHARI_MSGS_PUBLIC_TYPE
#endif
#endif  // SAPHARI_MSGS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:02:24
// Copyright 2019-2020 The MathWorks, Inc.
