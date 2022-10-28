#ifndef ACKERMANN_MSGS__VISIBILITY_CONTROL_H_
#define ACKERMANN_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ACKERMANN_MSGS_EXPORT __attribute__ ((dllexport))
    #define ACKERMANN_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define ACKERMANN_MSGS_EXPORT __declspec(dllexport)
    #define ACKERMANN_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef ACKERMANN_MSGS_BUILDING_LIBRARY
    #define ACKERMANN_MSGS_PUBLIC ACKERMANN_MSGS_EXPORT
  #else
    #define ACKERMANN_MSGS_PUBLIC ACKERMANN_MSGS_IMPORT
  #endif
  #define ACKERMANN_MSGS_PUBLIC_TYPE ACKERMANN_MSGS_PUBLIC
  #define ACKERMANN_MSGS_LOCAL
#else
  #define ACKERMANN_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define ACKERMANN_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define ACKERMANN_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define ACKERMANN_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ACKERMANN_MSGS_PUBLIC
    #define ACKERMANN_MSGS_LOCAL
  #endif
  #define ACKERMANN_MSGS_PUBLIC_TYPE
#endif
#endif  // ACKERMANN_MSGS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 17:57:54
// Copyright 2019-2020 The MathWorks, Inc.
