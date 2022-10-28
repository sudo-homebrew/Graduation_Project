#ifndef JACO_MSGS__VISIBILITY_CONTROL_H_
#define JACO_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define JACO_MSGS_EXPORT __attribute__ ((dllexport))
    #define JACO_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define JACO_MSGS_EXPORT __declspec(dllexport)
    #define JACO_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef JACO_MSGS_BUILDING_LIBRARY
    #define JACO_MSGS_PUBLIC JACO_MSGS_EXPORT
  #else
    #define JACO_MSGS_PUBLIC JACO_MSGS_IMPORT
  #endif
  #define JACO_MSGS_PUBLIC_TYPE JACO_MSGS_PUBLIC
  #define JACO_MSGS_LOCAL
#else
  #define JACO_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define JACO_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define JACO_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define JACO_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define JACO_MSGS_PUBLIC
    #define JACO_MSGS_LOCAL
  #endif
  #define JACO_MSGS_PUBLIC_TYPE
#endif
#endif  // JACO_MSGS__VISIBILITY_CONTROL_H_
// Generated 12-Apr-2020 11:46:35
// Copyright 2019-2020 The MathWorks, Inc.
