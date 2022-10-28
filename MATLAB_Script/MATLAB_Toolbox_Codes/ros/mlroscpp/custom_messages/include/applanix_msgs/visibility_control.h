#ifndef APPLANIX_MSGS__VISIBILITY_CONTROL_H_
#define APPLANIX_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define APPLANIX_MSGS_EXPORT __attribute__ ((dllexport))
    #define APPLANIX_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define APPLANIX_MSGS_EXPORT __declspec(dllexport)
    #define APPLANIX_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef APPLANIX_MSGS_BUILDING_LIBRARY
    #define APPLANIX_MSGS_PUBLIC APPLANIX_MSGS_EXPORT
  #else
    #define APPLANIX_MSGS_PUBLIC APPLANIX_MSGS_IMPORT
  #endif
  #define APPLANIX_MSGS_PUBLIC_TYPE APPLANIX_MSGS_PUBLIC
  #define APPLANIX_MSGS_LOCAL
#else
  #define APPLANIX_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define APPLANIX_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define APPLANIX_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define APPLANIX_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define APPLANIX_MSGS_PUBLIC
    #define APPLANIX_MSGS_LOCAL
  #endif
  #define APPLANIX_MSGS_PUBLIC_TYPE
#endif
#endif  // APPLANIX_MSGS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 17:58:06
// Copyright 2019-2020 The MathWorks, Inc.
