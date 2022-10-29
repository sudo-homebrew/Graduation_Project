#ifndef MANIPULATION_MSGS__VISIBILITY_CONTROL_H_
#define MANIPULATION_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MANIPULATION_MSGS_EXPORT __attribute__ ((dllexport))
    #define MANIPULATION_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define MANIPULATION_MSGS_EXPORT __declspec(dllexport)
    #define MANIPULATION_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef MANIPULATION_MSGS_BUILDING_LIBRARY
    #define MANIPULATION_MSGS_PUBLIC MANIPULATION_MSGS_EXPORT
  #else
    #define MANIPULATION_MSGS_PUBLIC MANIPULATION_MSGS_IMPORT
  #endif
  #define MANIPULATION_MSGS_PUBLIC_TYPE MANIPULATION_MSGS_PUBLIC
  #define MANIPULATION_MSGS_LOCAL
#else
  #define MANIPULATION_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define MANIPULATION_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define MANIPULATION_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define MANIPULATION_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MANIPULATION_MSGS_PUBLIC
    #define MANIPULATION_MSGS_LOCAL
  #endif
  #define MANIPULATION_MSGS_PUBLIC_TYPE
#endif
#endif  // MANIPULATION_MSGS__VISIBILITY_CONTROL_H_
// Generated 12-Apr-2020 11:46:38
// Copyright 2019-2020 The MathWorks, Inc.
