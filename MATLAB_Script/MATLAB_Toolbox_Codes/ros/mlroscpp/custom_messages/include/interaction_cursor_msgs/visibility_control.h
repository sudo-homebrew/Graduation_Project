#ifndef INTERACTION_CURSOR_MSGS__VISIBILITY_CONTROL_H_
#define INTERACTION_CURSOR_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define INTERACTION_CURSOR_MSGS_EXPORT __attribute__ ((dllexport))
    #define INTERACTION_CURSOR_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define INTERACTION_CURSOR_MSGS_EXPORT __declspec(dllexport)
    #define INTERACTION_CURSOR_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef INTERACTION_CURSOR_MSGS_BUILDING_LIBRARY
    #define INTERACTION_CURSOR_MSGS_PUBLIC INTERACTION_CURSOR_MSGS_EXPORT
  #else
    #define INTERACTION_CURSOR_MSGS_PUBLIC INTERACTION_CURSOR_MSGS_IMPORT
  #endif
  #define INTERACTION_CURSOR_MSGS_PUBLIC_TYPE INTERACTION_CURSOR_MSGS_PUBLIC
  #define INTERACTION_CURSOR_MSGS_LOCAL
#else
  #define INTERACTION_CURSOR_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define INTERACTION_CURSOR_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define INTERACTION_CURSOR_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define INTERACTION_CURSOR_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define INTERACTION_CURSOR_MSGS_PUBLIC
    #define INTERACTION_CURSOR_MSGS_LOCAL
  #endif
  #define INTERACTION_CURSOR_MSGS_PUBLIC_TYPE
#endif
#endif  // INTERACTION_CURSOR_MSGS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:00:09
// Copyright 2019-2020 The MathWorks, Inc.