#ifndef INDUSTRIAL_MSGS__VISIBILITY_CONTROL_H_
#define INDUSTRIAL_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define INDUSTRIAL_MSGS_EXPORT __attribute__ ((dllexport))
    #define INDUSTRIAL_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define INDUSTRIAL_MSGS_EXPORT __declspec(dllexport)
    #define INDUSTRIAL_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef INDUSTRIAL_MSGS_BUILDING_LIBRARY
    #define INDUSTRIAL_MSGS_PUBLIC INDUSTRIAL_MSGS_EXPORT
  #else
    #define INDUSTRIAL_MSGS_PUBLIC INDUSTRIAL_MSGS_IMPORT
  #endif
  #define INDUSTRIAL_MSGS_PUBLIC_TYPE INDUSTRIAL_MSGS_PUBLIC
  #define INDUSTRIAL_MSGS_LOCAL
#else
  #define INDUSTRIAL_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define INDUSTRIAL_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define INDUSTRIAL_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define INDUSTRIAL_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define INDUSTRIAL_MSGS_PUBLIC
    #define INDUSTRIAL_MSGS_LOCAL
  #endif
  #define INDUSTRIAL_MSGS_PUBLIC_TYPE
#endif
#endif  // INDUSTRIAL_MSGS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:00:07
// Copyright 2019-2020 The MathWorks, Inc.
