#ifndef GPS_COMMON__VISIBILITY_CONTROL_H_
#define GPS_COMMON__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GPS_COMMON_EXPORT __attribute__ ((dllexport))
    #define GPS_COMMON_IMPORT __attribute__ ((dllimport))
  #else
    #define GPS_COMMON_EXPORT __declspec(dllexport)
    #define GPS_COMMON_IMPORT __declspec(dllimport)
  #endif
  #ifdef GPS_COMMON_BUILDING_LIBRARY
    #define GPS_COMMON_PUBLIC GPS_COMMON_EXPORT
  #else
    #define GPS_COMMON_PUBLIC GPS_COMMON_IMPORT
  #endif
  #define GPS_COMMON_PUBLIC_TYPE GPS_COMMON_PUBLIC
  #define GPS_COMMON_LOCAL
#else
  #define GPS_COMMON_EXPORT __attribute__ ((visibility("default")))
  #define GPS_COMMON_IMPORT
  #if __GNUC__ >= 4
    #define GPS_COMMON_PUBLIC __attribute__ ((visibility("default")))
    #define GPS_COMMON_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GPS_COMMON_PUBLIC
    #define GPS_COMMON_LOCAL
  #endif
  #define GPS_COMMON_PUBLIC_TYPE
#endif
#endif  // GPS_COMMON__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 17:59:38
// Copyright 2019-2020 The MathWorks, Inc.
