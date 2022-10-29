#ifndef NAV2D_MSGS__VISIBILITY_CONTROL_H_
#define NAV2D_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define NAV2D_MSGS_EXPORT __attribute__ ((dllexport))
    #define NAV2D_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define NAV2D_MSGS_EXPORT __declspec(dllexport)
    #define NAV2D_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef NAV2D_MSGS_BUILDING_LIBRARY
    #define NAV2D_MSGS_PUBLIC NAV2D_MSGS_EXPORT
  #else
    #define NAV2D_MSGS_PUBLIC NAV2D_MSGS_IMPORT
  #endif
  #define NAV2D_MSGS_PUBLIC_TYPE NAV2D_MSGS_PUBLIC
  #define NAV2D_MSGS_LOCAL
#else
  #define NAV2D_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define NAV2D_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define NAV2D_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define NAV2D_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define NAV2D_MSGS_PUBLIC
    #define NAV2D_MSGS_LOCAL
  #endif
  #define NAV2D_MSGS_PUBLIC_TYPE
#endif
#endif  // NAV2D_MSGS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:01:06
// Copyright 2019-2020 The MathWorks, Inc.
