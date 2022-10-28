#ifndef CLEARPATH_BASE__VISIBILITY_CONTROL_H_
#define CLEARPATH_BASE__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CLEARPATH_BASE_EXPORT __attribute__ ((dllexport))
    #define CLEARPATH_BASE_IMPORT __attribute__ ((dllimport))
  #else
    #define CLEARPATH_BASE_EXPORT __declspec(dllexport)
    #define CLEARPATH_BASE_IMPORT __declspec(dllimport)
  #endif
  #ifdef CLEARPATH_BASE_BUILDING_LIBRARY
    #define CLEARPATH_BASE_PUBLIC CLEARPATH_BASE_EXPORT
  #else
    #define CLEARPATH_BASE_PUBLIC CLEARPATH_BASE_IMPORT
  #endif
  #define CLEARPATH_BASE_PUBLIC_TYPE CLEARPATH_BASE_PUBLIC
  #define CLEARPATH_BASE_LOCAL
#else
  #define CLEARPATH_BASE_EXPORT __attribute__ ((visibility("default")))
  #define CLEARPATH_BASE_IMPORT
  #if __GNUC__ >= 4
    #define CLEARPATH_BASE_PUBLIC __attribute__ ((visibility("default")))
    #define CLEARPATH_BASE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CLEARPATH_BASE_PUBLIC
    #define CLEARPATH_BASE_LOCAL
  #endif
  #define CLEARPATH_BASE_PUBLIC_TYPE
#endif
#endif  // CLEARPATH_BASE__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 12:35:55
// Copyright 2019-2020 The MathWorks, Inc.
