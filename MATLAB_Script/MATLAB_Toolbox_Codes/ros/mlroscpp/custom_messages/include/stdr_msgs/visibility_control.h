#ifndef STDR_MSGS__VISIBILITY_CONTROL_H_
#define STDR_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define STDR_MSGS_EXPORT __attribute__ ((dllexport))
    #define STDR_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define STDR_MSGS_EXPORT __declspec(dllexport)
    #define STDR_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef STDR_MSGS_BUILDING_LIBRARY
    #define STDR_MSGS_PUBLIC STDR_MSGS_EXPORT
  #else
    #define STDR_MSGS_PUBLIC STDR_MSGS_IMPORT
  #endif
  #define STDR_MSGS_PUBLIC_TYPE STDR_MSGS_PUBLIC
  #define STDR_MSGS_LOCAL
#else
  #define STDR_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define STDR_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define STDR_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define STDR_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define STDR_MSGS_PUBLIC
    #define STDR_MSGS_LOCAL
  #endif
  #define STDR_MSGS_PUBLIC_TYPE
#endif
#endif  // STDR_MSGS__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 14:37:40
// Copyright 2019-2020 The MathWorks, Inc.
