#ifndef SEGWAY_RMP__VISIBILITY_CONTROL_H_
#define SEGWAY_RMP__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SEGWAY_RMP_EXPORT __attribute__ ((dllexport))
    #define SEGWAY_RMP_IMPORT __attribute__ ((dllimport))
  #else
    #define SEGWAY_RMP_EXPORT __declspec(dllexport)
    #define SEGWAY_RMP_IMPORT __declspec(dllimport)
  #endif
  #ifdef SEGWAY_RMP_BUILDING_LIBRARY
    #define SEGWAY_RMP_PUBLIC SEGWAY_RMP_EXPORT
  #else
    #define SEGWAY_RMP_PUBLIC SEGWAY_RMP_IMPORT
  #endif
  #define SEGWAY_RMP_PUBLIC_TYPE SEGWAY_RMP_PUBLIC
  #define SEGWAY_RMP_LOCAL
#else
  #define SEGWAY_RMP_EXPORT __attribute__ ((visibility("default")))
  #define SEGWAY_RMP_IMPORT
  #if __GNUC__ >= 4
    #define SEGWAY_RMP_PUBLIC __attribute__ ((visibility("default")))
    #define SEGWAY_RMP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SEGWAY_RMP_PUBLIC
    #define SEGWAY_RMP_LOCAL
  #endif
  #define SEGWAY_RMP_PUBLIC_TYPE
#endif
#endif  // SEGWAY_RMP__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:02:32
// Copyright 2019-2020 The MathWorks, Inc.
