#ifndef CAPABILITIES__VISIBILITY_CONTROL_H_
#define CAPABILITIES__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CAPABILITIES_EXPORT __attribute__ ((dllexport))
    #define CAPABILITIES_IMPORT __attribute__ ((dllimport))
  #else
    #define CAPABILITIES_EXPORT __declspec(dllexport)
    #define CAPABILITIES_IMPORT __declspec(dllimport)
  #endif
  #ifdef CAPABILITIES_BUILDING_LIBRARY
    #define CAPABILITIES_PUBLIC CAPABILITIES_EXPORT
  #else
    #define CAPABILITIES_PUBLIC CAPABILITIES_IMPORT
  #endif
  #define CAPABILITIES_PUBLIC_TYPE CAPABILITIES_PUBLIC
  #define CAPABILITIES_LOCAL
#else
  #define CAPABILITIES_EXPORT __attribute__ ((visibility("default")))
  #define CAPABILITIES_IMPORT
  #if __GNUC__ >= 4
    #define CAPABILITIES_PUBLIC __attribute__ ((visibility("default")))
    #define CAPABILITIES_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CAPABILITIES_PUBLIC
    #define CAPABILITIES_LOCAL
  #endif
  #define CAPABILITIES_PUBLIC_TYPE
#endif
#endif  // CAPABILITIES__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 12:35:36
// Copyright 2019-2020 The MathWorks, Inc.
