#ifndef NMEA_MSGS__VISIBILITY_CONTROL_H_
#define NMEA_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define NMEA_MSGS_EXPORT __attribute__ ((dllexport))
    #define NMEA_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define NMEA_MSGS_EXPORT __declspec(dllexport)
    #define NMEA_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef NMEA_MSGS_BUILDING_LIBRARY
    #define NMEA_MSGS_PUBLIC NMEA_MSGS_EXPORT
  #else
    #define NMEA_MSGS_PUBLIC NMEA_MSGS_IMPORT
  #endif
  #define NMEA_MSGS_PUBLIC_TYPE NMEA_MSGS_PUBLIC
  #define NMEA_MSGS_LOCAL
#else
  #define NMEA_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define NMEA_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define NMEA_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define NMEA_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define NMEA_MSGS_PUBLIC
    #define NMEA_MSGS_LOCAL
  #endif
  #define NMEA_MSGS_PUBLIC_TYPE
#endif
#endif  // NMEA_MSGS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:01:14
// Copyright 2019-2020 The MathWorks, Inc.
