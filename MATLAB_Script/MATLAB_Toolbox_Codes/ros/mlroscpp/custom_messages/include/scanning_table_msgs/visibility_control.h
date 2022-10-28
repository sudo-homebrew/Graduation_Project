#ifndef SCANNING_TABLE_MSGS__VISIBILITY_CONTROL_H_
#define SCANNING_TABLE_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SCANNING_TABLE_MSGS_EXPORT __attribute__ ((dllexport))
    #define SCANNING_TABLE_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define SCANNING_TABLE_MSGS_EXPORT __declspec(dllexport)
    #define SCANNING_TABLE_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef SCANNING_TABLE_MSGS_BUILDING_LIBRARY
    #define SCANNING_TABLE_MSGS_PUBLIC SCANNING_TABLE_MSGS_EXPORT
  #else
    #define SCANNING_TABLE_MSGS_PUBLIC SCANNING_TABLE_MSGS_IMPORT
  #endif
  #define SCANNING_TABLE_MSGS_PUBLIC_TYPE SCANNING_TABLE_MSGS_PUBLIC
  #define SCANNING_TABLE_MSGS_LOCAL
#else
  #define SCANNING_TABLE_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define SCANNING_TABLE_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define SCANNING_TABLE_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define SCANNING_TABLE_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SCANNING_TABLE_MSGS_PUBLIC
    #define SCANNING_TABLE_MSGS_LOCAL
  #endif
  #define SCANNING_TABLE_MSGS_PUBLIC_TYPE
#endif
#endif  // SCANNING_TABLE_MSGS__VISIBILITY_CONTROL_H_
// Generated 11-Apr-2020 16:38:21
// Copyright 2019-2020 The MathWorks, Inc.
