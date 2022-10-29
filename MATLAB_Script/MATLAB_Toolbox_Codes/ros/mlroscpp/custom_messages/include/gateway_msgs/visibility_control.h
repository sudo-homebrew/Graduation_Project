#ifndef GATEWAY_MSGS__VISIBILITY_CONTROL_H_
#define GATEWAY_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GATEWAY_MSGS_EXPORT __attribute__ ((dllexport))
    #define GATEWAY_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define GATEWAY_MSGS_EXPORT __declspec(dllexport)
    #define GATEWAY_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef GATEWAY_MSGS_BUILDING_LIBRARY
    #define GATEWAY_MSGS_PUBLIC GATEWAY_MSGS_EXPORT
  #else
    #define GATEWAY_MSGS_PUBLIC GATEWAY_MSGS_IMPORT
  #endif
  #define GATEWAY_MSGS_PUBLIC_TYPE GATEWAY_MSGS_PUBLIC
  #define GATEWAY_MSGS_LOCAL
#else
  #define GATEWAY_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define GATEWAY_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define GATEWAY_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define GATEWAY_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GATEWAY_MSGS_PUBLIC
    #define GATEWAY_MSGS_LOCAL
  #endif
  #define GATEWAY_MSGS_PUBLIC_TYPE
#endif
#endif  // GATEWAY_MSGS__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 13:03:16
// Copyright 2019-2020 The MathWorks, Inc.
