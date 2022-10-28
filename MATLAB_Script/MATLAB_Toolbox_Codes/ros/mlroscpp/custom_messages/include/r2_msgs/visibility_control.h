#ifndef R2_MSGS__VISIBILITY_CONTROL_H_
#define R2_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define R2_MSGS_EXPORT __attribute__ ((dllexport))
    #define R2_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define R2_MSGS_EXPORT __declspec(dllexport)
    #define R2_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef R2_MSGS_BUILDING_LIBRARY
    #define R2_MSGS_PUBLIC R2_MSGS_EXPORT
  #else
    #define R2_MSGS_PUBLIC R2_MSGS_IMPORT
  #endif
  #define R2_MSGS_PUBLIC_TYPE R2_MSGS_PUBLIC
  #define R2_MSGS_LOCAL
#else
  #define R2_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define R2_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define R2_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define R2_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define R2_MSGS_PUBLIC
    #define R2_MSGS_LOCAL
  #endif
  #define R2_MSGS_PUBLIC_TYPE
#endif
#endif  // R2_MSGS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:01:51
// Copyright 2019-2020 The MathWorks, Inc.
