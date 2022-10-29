#ifndef GEOGRAPHIC_MSGS__VISIBILITY_CONTROL_H_
#define GEOGRAPHIC_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GEOGRAPHIC_MSGS_EXPORT __attribute__ ((dllexport))
    #define GEOGRAPHIC_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define GEOGRAPHIC_MSGS_EXPORT __declspec(dllexport)
    #define GEOGRAPHIC_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef GEOGRAPHIC_MSGS_BUILDING_LIBRARY
    #define GEOGRAPHIC_MSGS_PUBLIC GEOGRAPHIC_MSGS_EXPORT
  #else
    #define GEOGRAPHIC_MSGS_PUBLIC GEOGRAPHIC_MSGS_IMPORT
  #endif
  #define GEOGRAPHIC_MSGS_PUBLIC_TYPE GEOGRAPHIC_MSGS_PUBLIC
  #define GEOGRAPHIC_MSGS_LOCAL
#else
  #define GEOGRAPHIC_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define GEOGRAPHIC_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define GEOGRAPHIC_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define GEOGRAPHIC_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GEOGRAPHIC_MSGS_PUBLIC
    #define GEOGRAPHIC_MSGS_LOCAL
  #endif
  #define GEOGRAPHIC_MSGS_PUBLIC_TYPE
#endif
#endif  // GEOGRAPHIC_MSGS__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 13:50:30
// Copyright 2019-2020 The MathWorks, Inc.
