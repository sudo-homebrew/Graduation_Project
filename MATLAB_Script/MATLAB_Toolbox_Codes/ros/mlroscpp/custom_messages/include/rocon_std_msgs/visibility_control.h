#ifndef ROCON_STD_MSGS__VISIBILITY_CONTROL_H_
#define ROCON_STD_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROCON_STD_MSGS_EXPORT __attribute__ ((dllexport))
    #define ROCON_STD_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define ROCON_STD_MSGS_EXPORT __declspec(dllexport)
    #define ROCON_STD_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROCON_STD_MSGS_BUILDING_LIBRARY
    #define ROCON_STD_MSGS_PUBLIC ROCON_STD_MSGS_EXPORT
  #else
    #define ROCON_STD_MSGS_PUBLIC ROCON_STD_MSGS_IMPORT
  #endif
  #define ROCON_STD_MSGS_PUBLIC_TYPE ROCON_STD_MSGS_PUBLIC
  #define ROCON_STD_MSGS_LOCAL
#else
  #define ROCON_STD_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define ROCON_STD_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define ROCON_STD_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define ROCON_STD_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROCON_STD_MSGS_PUBLIC
    #define ROCON_STD_MSGS_LOCAL
  #endif
  #define ROCON_STD_MSGS_PUBLIC_TYPE
#endif
#endif  // ROCON_STD_MSGS__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 14:35:55
// Copyright 2019-2020 The MathWorks, Inc.
