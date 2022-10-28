#ifndef WIFI_DDWRT__VISIBILITY_CONTROL_H_
#define WIFI_DDWRT__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define WIFI_DDWRT_EXPORT __attribute__ ((dllexport))
    #define WIFI_DDWRT_IMPORT __attribute__ ((dllimport))
  #else
    #define WIFI_DDWRT_EXPORT __declspec(dllexport)
    #define WIFI_DDWRT_IMPORT __declspec(dllimport)
  #endif
  #ifdef WIFI_DDWRT_BUILDING_LIBRARY
    #define WIFI_DDWRT_PUBLIC WIFI_DDWRT_EXPORT
  #else
    #define WIFI_DDWRT_PUBLIC WIFI_DDWRT_IMPORT
  #endif
  #define WIFI_DDWRT_PUBLIC_TYPE WIFI_DDWRT_PUBLIC
  #define WIFI_DDWRT_LOCAL
#else
  #define WIFI_DDWRT_EXPORT __attribute__ ((visibility("default")))
  #define WIFI_DDWRT_IMPORT
  #if __GNUC__ >= 4
    #define WIFI_DDWRT_PUBLIC __attribute__ ((visibility("default")))
    #define WIFI_DDWRT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define WIFI_DDWRT_PUBLIC
    #define WIFI_DDWRT_LOCAL
  #endif
  #define WIFI_DDWRT_PUBLIC_TYPE
#endif
#endif  // WIFI_DDWRT__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:03:39
// Copyright 2019-2020 The MathWorks, Inc.
