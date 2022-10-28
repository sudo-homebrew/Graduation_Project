#ifndef EXPLORER__VISIBILITY_CONTROL_H_
#define EXPLORER__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define EXPLORER_EXPORT __attribute__ ((dllexport))
    #define EXPLORER_IMPORT __attribute__ ((dllimport))
  #else
    #define EXPLORER_EXPORT __declspec(dllexport)
    #define EXPLORER_IMPORT __declspec(dllimport)
  #endif
  #ifdef EXPLORER_BUILDING_LIBRARY
    #define EXPLORER_PUBLIC EXPLORER_EXPORT
  #else
    #define EXPLORER_PUBLIC EXPLORER_IMPORT
  #endif
  #define EXPLORER_PUBLIC_TYPE EXPLORER_PUBLIC
  #define EXPLORER_LOCAL
#else
  #define EXPLORER_EXPORT __attribute__ ((visibility("default")))
  #define EXPLORER_IMPORT
  #if __GNUC__ >= 4
    #define EXPLORER_PUBLIC __attribute__ ((visibility("default")))
    #define EXPLORER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define EXPLORER_PUBLIC
    #define EXPLORER_LOCAL
  #endif
  #define EXPLORER_PUBLIC_TYPE
#endif
#endif  // EXPLORER__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 17:59:23
// Copyright 2019-2020 The MathWorks, Inc.
