#ifndef COB_PHIDGETS__VISIBILITY_CONTROL_H_
#define COB_PHIDGETS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define COB_PHIDGETS_EXPORT __attribute__ ((dllexport))
    #define COB_PHIDGETS_IMPORT __attribute__ ((dllimport))
  #else
    #define COB_PHIDGETS_EXPORT __declspec(dllexport)
    #define COB_PHIDGETS_IMPORT __declspec(dllimport)
  #endif
  #ifdef COB_PHIDGETS_BUILDING_LIBRARY
    #define COB_PHIDGETS_PUBLIC COB_PHIDGETS_EXPORT
  #else
    #define COB_PHIDGETS_PUBLIC COB_PHIDGETS_IMPORT
  #endif
  #define COB_PHIDGETS_PUBLIC_TYPE COB_PHIDGETS_PUBLIC
  #define COB_PHIDGETS_LOCAL
#else
  #define COB_PHIDGETS_EXPORT __attribute__ ((visibility("default")))
  #define COB_PHIDGETS_IMPORT
  #if __GNUC__ >= 4
    #define COB_PHIDGETS_PUBLIC __attribute__ ((visibility("default")))
    #define COB_PHIDGETS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define COB_PHIDGETS_PUBLIC
    #define COB_PHIDGETS_LOCAL
  #endif
  #define COB_PHIDGETS_PUBLIC_TYPE
#endif
#endif  // COB_PHIDGETS__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 12:37:00
// Copyright 2019-2020 The MathWorks, Inc.
