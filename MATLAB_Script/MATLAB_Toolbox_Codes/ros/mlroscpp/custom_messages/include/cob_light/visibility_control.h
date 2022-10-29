#ifndef COB_LIGHT__VISIBILITY_CONTROL_H_
#define COB_LIGHT__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define COB_LIGHT_EXPORT __attribute__ ((dllexport))
    #define COB_LIGHT_IMPORT __attribute__ ((dllimport))
  #else
    #define COB_LIGHT_EXPORT __declspec(dllexport)
    #define COB_LIGHT_IMPORT __declspec(dllimport)
  #endif
  #ifdef COB_LIGHT_BUILDING_LIBRARY
    #define COB_LIGHT_PUBLIC COB_LIGHT_EXPORT
  #else
    #define COB_LIGHT_PUBLIC COB_LIGHT_IMPORT
  #endif
  #define COB_LIGHT_PUBLIC_TYPE COB_LIGHT_PUBLIC
  #define COB_LIGHT_LOCAL
#else
  #define COB_LIGHT_EXPORT __attribute__ ((visibility("default")))
  #define COB_LIGHT_IMPORT
  #if __GNUC__ >= 4
    #define COB_LIGHT_PUBLIC __attribute__ ((visibility("default")))
    #define COB_LIGHT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define COB_LIGHT_PUBLIC
    #define COB_LIGHT_LOCAL
  #endif
  #define COB_LIGHT_PUBLIC_TYPE
#endif
#endif  // COB_LIGHT__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 12:36:33
// Copyright 2019-2020 The MathWorks, Inc.
