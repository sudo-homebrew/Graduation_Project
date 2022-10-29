#ifndef COB_SRVS__VISIBILITY_CONTROL_H_
#define COB_SRVS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define COB_SRVS_EXPORT __attribute__ ((dllexport))
    #define COB_SRVS_IMPORT __attribute__ ((dllimport))
  #else
    #define COB_SRVS_EXPORT __declspec(dllexport)
    #define COB_SRVS_IMPORT __declspec(dllimport)
  #endif
  #ifdef COB_SRVS_BUILDING_LIBRARY
    #define COB_SRVS_PUBLIC COB_SRVS_EXPORT
  #else
    #define COB_SRVS_PUBLIC COB_SRVS_IMPORT
  #endif
  #define COB_SRVS_PUBLIC_TYPE COB_SRVS_PUBLIC
  #define COB_SRVS_LOCAL
#else
  #define COB_SRVS_EXPORT __attribute__ ((visibility("default")))
  #define COB_SRVS_IMPORT
  #if __GNUC__ >= 4
    #define COB_SRVS_PUBLIC __attribute__ ((visibility("default")))
    #define COB_SRVS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define COB_SRVS_PUBLIC
    #define COB_SRVS_LOCAL
  #endif
  #define COB_SRVS_PUBLIC_TYPE
#endif
#endif  // COB_SRVS__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 12:37:42
// Copyright 2019-2020 The MathWorks, Inc.
