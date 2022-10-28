#ifndef ROBOTEQ_MSGS__VISIBILITY_CONTROL_H_
#define ROBOTEQ_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROBOTEQ_MSGS_EXPORT __attribute__ ((dllexport))
    #define ROBOTEQ_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define ROBOTEQ_MSGS_EXPORT __declspec(dllexport)
    #define ROBOTEQ_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROBOTEQ_MSGS_BUILDING_LIBRARY
    #define ROBOTEQ_MSGS_PUBLIC ROBOTEQ_MSGS_EXPORT
  #else
    #define ROBOTEQ_MSGS_PUBLIC ROBOTEQ_MSGS_IMPORT
  #endif
  #define ROBOTEQ_MSGS_PUBLIC_TYPE ROBOTEQ_MSGS_PUBLIC
  #define ROBOTEQ_MSGS_LOCAL
#else
  #define ROBOTEQ_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define ROBOTEQ_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define ROBOTEQ_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define ROBOTEQ_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROBOTEQ_MSGS_PUBLIC
    #define ROBOTEQ_MSGS_LOCAL
  #endif
  #define ROBOTEQ_MSGS_PUBLIC_TYPE
#endif
#endif  // ROBOTEQ_MSGS__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 14:27:23
// Copyright 2019-2020 The MathWorks, Inc.
