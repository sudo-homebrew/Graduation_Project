#ifndef LEAP_MOTION__VISIBILITY_CONTROL_H_
#define LEAP_MOTION__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define LEAP_MOTION_EXPORT __attribute__ ((dllexport))
    #define LEAP_MOTION_IMPORT __attribute__ ((dllimport))
  #else
    #define LEAP_MOTION_EXPORT __declspec(dllexport)
    #define LEAP_MOTION_IMPORT __declspec(dllimport)
  #endif
  #ifdef LEAP_MOTION_BUILDING_LIBRARY
    #define LEAP_MOTION_PUBLIC LEAP_MOTION_EXPORT
  #else
    #define LEAP_MOTION_PUBLIC LEAP_MOTION_IMPORT
  #endif
  #define LEAP_MOTION_PUBLIC_TYPE LEAP_MOTION_PUBLIC
  #define LEAP_MOTION_LOCAL
#else
  #define LEAP_MOTION_EXPORT __attribute__ ((visibility("default")))
  #define LEAP_MOTION_IMPORT
  #if __GNUC__ >= 4
    #define LEAP_MOTION_PUBLIC __attribute__ ((visibility("default")))
    #define LEAP_MOTION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define LEAP_MOTION_PUBLIC
    #define LEAP_MOTION_LOCAL
  #endif
  #define LEAP_MOTION_PUBLIC_TYPE
#endif
#endif  // LEAP_MOTION__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 14:01:40
// Copyright 2019-2020 The MathWorks, Inc.
