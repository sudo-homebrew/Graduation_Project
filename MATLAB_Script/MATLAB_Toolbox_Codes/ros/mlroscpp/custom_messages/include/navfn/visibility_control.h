#ifndef NAVFN__VISIBILITY_CONTROL_H_
#define NAVFN__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define NAVFN_EXPORT __attribute__ ((dllexport))
    #define NAVFN_IMPORT __attribute__ ((dllimport))
  #else
    #define NAVFN_EXPORT __declspec(dllexport)
    #define NAVFN_IMPORT __declspec(dllimport)
  #endif
  #ifdef NAVFN_BUILDING_LIBRARY
    #define NAVFN_PUBLIC NAVFN_EXPORT
  #else
    #define NAVFN_PUBLIC NAVFN_IMPORT
  #endif
  #define NAVFN_PUBLIC_TYPE NAVFN_PUBLIC
  #define NAVFN_LOCAL
#else
  #define NAVFN_EXPORT __attribute__ ((visibility("default")))
  #define NAVFN_IMPORT
  #if __GNUC__ >= 4
    #define NAVFN_PUBLIC __attribute__ ((visibility("default")))
    #define NAVFN_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define NAVFN_PUBLIC
    #define NAVFN_LOCAL
  #endif
  #define NAVFN_PUBLIC_TYPE
#endif
#endif  // NAVFN__VISIBILITY_CONTROL_H_
// Generated 25-Mar-2020 21:51:31
// Copyright 2019-2020 The MathWorks, Inc.
