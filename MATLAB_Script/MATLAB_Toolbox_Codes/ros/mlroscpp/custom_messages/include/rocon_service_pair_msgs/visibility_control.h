#ifndef ROCON_SERVICE_PAIR_MSGS__VISIBILITY_CONTROL_H_
#define ROCON_SERVICE_PAIR_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROCON_SERVICE_PAIR_MSGS_EXPORT __attribute__ ((dllexport))
    #define ROCON_SERVICE_PAIR_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define ROCON_SERVICE_PAIR_MSGS_EXPORT __declspec(dllexport)
    #define ROCON_SERVICE_PAIR_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROCON_SERVICE_PAIR_MSGS_BUILDING_LIBRARY
    #define ROCON_SERVICE_PAIR_MSGS_PUBLIC ROCON_SERVICE_PAIR_MSGS_EXPORT
  #else
    #define ROCON_SERVICE_PAIR_MSGS_PUBLIC ROCON_SERVICE_PAIR_MSGS_IMPORT
  #endif
  #define ROCON_SERVICE_PAIR_MSGS_PUBLIC_TYPE ROCON_SERVICE_PAIR_MSGS_PUBLIC
  #define ROCON_SERVICE_PAIR_MSGS_LOCAL
#else
  #define ROCON_SERVICE_PAIR_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define ROCON_SERVICE_PAIR_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define ROCON_SERVICE_PAIR_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define ROCON_SERVICE_PAIR_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROCON_SERVICE_PAIR_MSGS_PUBLIC
    #define ROCON_SERVICE_PAIR_MSGS_LOCAL
  #endif
  #define ROCON_SERVICE_PAIR_MSGS_PUBLIC_TYPE
#endif
#endif  // ROCON_SERVICE_PAIR_MSGS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:02:06
// Copyright 2019-2020 The MathWorks, Inc.
