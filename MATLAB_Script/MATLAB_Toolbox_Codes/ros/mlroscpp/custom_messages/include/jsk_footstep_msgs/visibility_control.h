#ifndef JSK_FOOTSTEP_MSGS__VISIBILITY_CONTROL_H_
#define JSK_FOOTSTEP_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define JSK_FOOTSTEP_MSGS_EXPORT __attribute__ ((dllexport))
    #define JSK_FOOTSTEP_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define JSK_FOOTSTEP_MSGS_EXPORT __declspec(dllexport)
    #define JSK_FOOTSTEP_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef JSK_FOOTSTEP_MSGS_BUILDING_LIBRARY
    #define JSK_FOOTSTEP_MSGS_PUBLIC JSK_FOOTSTEP_MSGS_EXPORT
  #else
    #define JSK_FOOTSTEP_MSGS_PUBLIC JSK_FOOTSTEP_MSGS_IMPORT
  #endif
  #define JSK_FOOTSTEP_MSGS_PUBLIC_TYPE JSK_FOOTSTEP_MSGS_PUBLIC
  #define JSK_FOOTSTEP_MSGS_LOCAL
#else
  #define JSK_FOOTSTEP_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define JSK_FOOTSTEP_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define JSK_FOOTSTEP_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define JSK_FOOTSTEP_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define JSK_FOOTSTEP_MSGS_PUBLIC
    #define JSK_FOOTSTEP_MSGS_LOCAL
  #endif
  #define JSK_FOOTSTEP_MSGS_PUBLIC_TYPE
#endif
#endif  // JSK_FOOTSTEP_MSGS__VISIBILITY_CONTROL_H_
// Generated 12-Apr-2020 11:46:36
// Copyright 2019-2020 The MathWorks, Inc.
