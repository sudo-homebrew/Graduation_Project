#ifndef ROSEUS__VISIBILITY_CONTROL_H_
#define ROSEUS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSEUS_EXPORT __attribute__ ((dllexport))
    #define ROSEUS_IMPORT __attribute__ ((dllimport))
  #else
    #define ROSEUS_EXPORT __declspec(dllexport)
    #define ROSEUS_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROSEUS_BUILDING_LIBRARY
    #define ROSEUS_PUBLIC ROSEUS_EXPORT
  #else
    #define ROSEUS_PUBLIC ROSEUS_IMPORT
  #endif
  #define ROSEUS_PUBLIC_TYPE ROSEUS_PUBLIC
  #define ROSEUS_LOCAL
#else
  #define ROSEUS_EXPORT __attribute__ ((visibility("default")))
  #define ROSEUS_IMPORT
  #if __GNUC__ >= 4
    #define ROSEUS_PUBLIC __attribute__ ((visibility("default")))
    #define ROSEUS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROSEUS_PUBLIC
    #define ROSEUS_LOCAL
  #endif
  #define ROSEUS_PUBLIC_TYPE
#endif
#endif  // ROSEUS__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 14:36:02
// Copyright 2019-2020 The MathWorks, Inc.
