#ifndef ZEROCONF_MSGS__VISIBILITY_CONTROL_H_
#define ZEROCONF_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ZEROCONF_MSGS_EXPORT __attribute__ ((dllexport))
    #define ZEROCONF_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define ZEROCONF_MSGS_EXPORT __declspec(dllexport)
    #define ZEROCONF_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef ZEROCONF_MSGS_BUILDING_LIBRARY
    #define ZEROCONF_MSGS_PUBLIC ZEROCONF_MSGS_EXPORT
  #else
    #define ZEROCONF_MSGS_PUBLIC ZEROCONF_MSGS_IMPORT
  #endif
  #define ZEROCONF_MSGS_PUBLIC_TYPE ZEROCONF_MSGS_PUBLIC
  #define ZEROCONF_MSGS_LOCAL
#else
  #define ZEROCONF_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define ZEROCONF_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define ZEROCONF_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define ZEROCONF_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ZEROCONF_MSGS_PUBLIC
    #define ZEROCONF_MSGS_LOCAL
  #endif
  #define ZEROCONF_MSGS_PUBLIC_TYPE
#endif
#endif  // ZEROCONF_MSGS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:03:45
// Copyright 2019-2020 The MathWorks, Inc.
