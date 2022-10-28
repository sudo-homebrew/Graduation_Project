#ifndef UUID_MSGS__VISIBILITY_CONTROL_H_
#define UUID_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define UUID_MSGS_EXPORT __attribute__ ((dllexport))
    #define UUID_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define UUID_MSGS_EXPORT __declspec(dllexport)
    #define UUID_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef UUID_MSGS_BUILDING_LIBRARY
    #define UUID_MSGS_PUBLIC UUID_MSGS_EXPORT
  #else
    #define UUID_MSGS_PUBLIC UUID_MSGS_IMPORT
  #endif
  #define UUID_MSGS_PUBLIC_TYPE UUID_MSGS_PUBLIC
  #define UUID_MSGS_LOCAL
#else
  #define UUID_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define UUID_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define UUID_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define UUID_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define UUID_MSGS_PUBLIC
    #define UUID_MSGS_LOCAL
  #endif
  #define UUID_MSGS_PUBLIC_TYPE
#endif
#endif  // UUID_MSGS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:03:23
// Copyright 2019-2020 The MathWorks, Inc.
