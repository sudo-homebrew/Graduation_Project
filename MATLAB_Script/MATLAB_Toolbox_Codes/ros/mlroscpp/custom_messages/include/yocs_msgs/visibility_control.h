#ifndef YOCS_MSGS__VISIBILITY_CONTROL_H_
#define YOCS_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define YOCS_MSGS_EXPORT __attribute__ ((dllexport))
    #define YOCS_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define YOCS_MSGS_EXPORT __declspec(dllexport)
    #define YOCS_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef YOCS_MSGS_BUILDING_LIBRARY
    #define YOCS_MSGS_PUBLIC YOCS_MSGS_EXPORT
  #else
    #define YOCS_MSGS_PUBLIC YOCS_MSGS_IMPORT
  #endif
  #define YOCS_MSGS_PUBLIC_TYPE YOCS_MSGS_PUBLIC
  #define YOCS_MSGS_LOCAL
#else
  #define YOCS_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define YOCS_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define YOCS_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define YOCS_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define YOCS_MSGS_PUBLIC
    #define YOCS_MSGS_LOCAL
  #endif
  #define YOCS_MSGS_PUBLIC_TYPE
#endif
#endif  // YOCS_MSGS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:03:43
// Copyright 2019-2020 The MathWorks, Inc.
