#ifndef SR_RONEX_MSGS__VISIBILITY_CONTROL_H_
#define SR_RONEX_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SR_RONEX_MSGS_EXPORT __attribute__ ((dllexport))
    #define SR_RONEX_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define SR_RONEX_MSGS_EXPORT __declspec(dllexport)
    #define SR_RONEX_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef SR_RONEX_MSGS_BUILDING_LIBRARY
    #define SR_RONEX_MSGS_PUBLIC SR_RONEX_MSGS_EXPORT
  #else
    #define SR_RONEX_MSGS_PUBLIC SR_RONEX_MSGS_IMPORT
  #endif
  #define SR_RONEX_MSGS_PUBLIC_TYPE SR_RONEX_MSGS_PUBLIC
  #define SR_RONEX_MSGS_LOCAL
#else
  #define SR_RONEX_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define SR_RONEX_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define SR_RONEX_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define SR_RONEX_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SR_RONEX_MSGS_PUBLIC
    #define SR_RONEX_MSGS_LOCAL
  #endif
  #define SR_RONEX_MSGS_PUBLIC_TYPE
#endif
#endif  // SR_RONEX_MSGS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:02:52
// Copyright 2019-2020 The MathWorks, Inc.
