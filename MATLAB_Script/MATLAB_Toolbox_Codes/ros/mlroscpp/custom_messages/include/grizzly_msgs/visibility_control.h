#ifndef GRIZZLY_MSGS__VISIBILITY_CONTROL_H_
#define GRIZZLY_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GRIZZLY_MSGS_EXPORT __attribute__ ((dllexport))
    #define GRIZZLY_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define GRIZZLY_MSGS_EXPORT __declspec(dllexport)
    #define GRIZZLY_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef GRIZZLY_MSGS_BUILDING_LIBRARY
    #define GRIZZLY_MSGS_PUBLIC GRIZZLY_MSGS_EXPORT
  #else
    #define GRIZZLY_MSGS_PUBLIC GRIZZLY_MSGS_IMPORT
  #endif
  #define GRIZZLY_MSGS_PUBLIC_TYPE GRIZZLY_MSGS_PUBLIC
  #define GRIZZLY_MSGS_LOCAL
#else
  #define GRIZZLY_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define GRIZZLY_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define GRIZZLY_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define GRIZZLY_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GRIZZLY_MSGS_PUBLIC
    #define GRIZZLY_MSGS_LOCAL
  #endif
  #define GRIZZLY_MSGS_PUBLIC_TYPE
#endif
#endif  // GRIZZLY_MSGS__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 13:50:37
// Copyright 2019-2020 The MathWorks, Inc.
