#ifndef UNIVERSAL_TELEOP__VISIBILITY_CONTROL_H_
#define UNIVERSAL_TELEOP__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define UNIVERSAL_TELEOP_EXPORT __attribute__ ((dllexport))
    #define UNIVERSAL_TELEOP_IMPORT __attribute__ ((dllimport))
  #else
    #define UNIVERSAL_TELEOP_EXPORT __declspec(dllexport)
    #define UNIVERSAL_TELEOP_IMPORT __declspec(dllimport)
  #endif
  #ifdef UNIVERSAL_TELEOP_BUILDING_LIBRARY
    #define UNIVERSAL_TELEOP_PUBLIC UNIVERSAL_TELEOP_EXPORT
  #else
    #define UNIVERSAL_TELEOP_PUBLIC UNIVERSAL_TELEOP_IMPORT
  #endif
  #define UNIVERSAL_TELEOP_PUBLIC_TYPE UNIVERSAL_TELEOP_PUBLIC
  #define UNIVERSAL_TELEOP_LOCAL
#else
  #define UNIVERSAL_TELEOP_EXPORT __attribute__ ((visibility("default")))
  #define UNIVERSAL_TELEOP_IMPORT
  #if __GNUC__ >= 4
    #define UNIVERSAL_TELEOP_PUBLIC __attribute__ ((visibility("default")))
    #define UNIVERSAL_TELEOP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define UNIVERSAL_TELEOP_PUBLIC
    #define UNIVERSAL_TELEOP_LOCAL
  #endif
  #define UNIVERSAL_TELEOP_PUBLIC_TYPE
#endif
#endif  // UNIVERSAL_TELEOP__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:03:21
// Copyright 2019-2020 The MathWorks, Inc.
