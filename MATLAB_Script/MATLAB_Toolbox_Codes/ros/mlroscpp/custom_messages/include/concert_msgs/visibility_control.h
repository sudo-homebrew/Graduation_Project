#ifndef CONCERT_MSGS__VISIBILITY_CONTROL_H_
#define CONCERT_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CONCERT_MSGS_EXPORT __attribute__ ((dllexport))
    #define CONCERT_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define CONCERT_MSGS_EXPORT __declspec(dllexport)
    #define CONCERT_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef CONCERT_MSGS_BUILDING_LIBRARY
    #define CONCERT_MSGS_PUBLIC CONCERT_MSGS_EXPORT
  #else
    #define CONCERT_MSGS_PUBLIC CONCERT_MSGS_IMPORT
  #endif
  #define CONCERT_MSGS_PUBLIC_TYPE CONCERT_MSGS_PUBLIC
  #define CONCERT_MSGS_LOCAL
#else
  #define CONCERT_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define CONCERT_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define CONCERT_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define CONCERT_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CONCERT_MSGS_PUBLIC
    #define CONCERT_MSGS_LOCAL
  #endif
  #define CONCERT_MSGS_PUBLIC_TYPE
#endif
#endif  // CONCERT_MSGS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 17:58:55
// Copyright 2019-2020 The MathWorks, Inc.
