#ifndef KOBUKI_MSGS__VISIBILITY_CONTROL_H_
#define KOBUKI_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define KOBUKI_MSGS_EXPORT __attribute__ ((dllexport))
    #define KOBUKI_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define KOBUKI_MSGS_EXPORT __declspec(dllexport)
    #define KOBUKI_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef KOBUKI_MSGS_BUILDING_LIBRARY
    #define KOBUKI_MSGS_PUBLIC KOBUKI_MSGS_EXPORT
  #else
    #define KOBUKI_MSGS_PUBLIC KOBUKI_MSGS_IMPORT
  #endif
  #define KOBUKI_MSGS_PUBLIC_TYPE KOBUKI_MSGS_PUBLIC
  #define KOBUKI_MSGS_LOCAL
#else
  #define KOBUKI_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define KOBUKI_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define KOBUKI_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define KOBUKI_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define KOBUKI_MSGS_PUBLIC
    #define KOBUKI_MSGS_LOCAL
  #endif
  #define KOBUKI_MSGS_PUBLIC_TYPE
#endif
#endif  // KOBUKI_MSGS__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 14:01:26
// Copyright 2019-2020 The MathWorks, Inc.
