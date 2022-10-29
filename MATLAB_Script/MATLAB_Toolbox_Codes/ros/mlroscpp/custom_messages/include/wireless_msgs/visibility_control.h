#ifndef WIRELESS_MSGS__VISIBILITY_CONTROL_H_
#define WIRELESS_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define WIRELESS_MSGS_EXPORT __attribute__ ((dllexport))
    #define WIRELESS_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define WIRELESS_MSGS_EXPORT __declspec(dllexport)
    #define WIRELESS_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef WIRELESS_MSGS_BUILDING_LIBRARY
    #define WIRELESS_MSGS_PUBLIC WIRELESS_MSGS_EXPORT
  #else
    #define WIRELESS_MSGS_PUBLIC WIRELESS_MSGS_IMPORT
  #endif
  #define WIRELESS_MSGS_PUBLIC_TYPE WIRELESS_MSGS_PUBLIC
  #define WIRELESS_MSGS_LOCAL
#else
  #define WIRELESS_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define WIRELESS_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define WIRELESS_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define WIRELESS_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define WIRELESS_MSGS_PUBLIC
    #define WIRELESS_MSGS_LOCAL
  #endif
  #define WIRELESS_MSGS_PUBLIC_TYPE
#endif
#endif  // WIRELESS_MSGS__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 14:37:50
// Copyright 2019-2020 The MathWorks, Inc.
