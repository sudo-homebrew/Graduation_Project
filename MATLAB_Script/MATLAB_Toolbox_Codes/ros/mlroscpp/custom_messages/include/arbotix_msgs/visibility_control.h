#ifndef ARBOTIX_MSGS__VISIBILITY_CONTROL_H_
#define ARBOTIX_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ARBOTIX_MSGS_EXPORT __attribute__ ((dllexport))
    #define ARBOTIX_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define ARBOTIX_MSGS_EXPORT __declspec(dllexport)
    #define ARBOTIX_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef ARBOTIX_MSGS_BUILDING_LIBRARY
    #define ARBOTIX_MSGS_PUBLIC ARBOTIX_MSGS_EXPORT
  #else
    #define ARBOTIX_MSGS_PUBLIC ARBOTIX_MSGS_IMPORT
  #endif
  #define ARBOTIX_MSGS_PUBLIC_TYPE ARBOTIX_MSGS_PUBLIC
  #define ARBOTIX_MSGS_LOCAL
#else
  #define ARBOTIX_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define ARBOTIX_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define ARBOTIX_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define ARBOTIX_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ARBOTIX_MSGS_PUBLIC
    #define ARBOTIX_MSGS_LOCAL
  #endif
  #define ARBOTIX_MSGS_PUBLIC_TYPE
#endif
#endif  // ARBOTIX_MSGS__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 12:34:37
// Copyright 2019-2020 The MathWorks, Inc.
