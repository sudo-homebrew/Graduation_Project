#ifndef ROSAUTH__VISIBILITY_CONTROL_H_
#define ROSAUTH__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSAUTH_EXPORT __attribute__ ((dllexport))
    #define ROSAUTH_IMPORT __attribute__ ((dllimport))
  #else
    #define ROSAUTH_EXPORT __declspec(dllexport)
    #define ROSAUTH_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROSAUTH_BUILDING_LIBRARY
    #define ROSAUTH_PUBLIC ROSAUTH_EXPORT
  #else
    #define ROSAUTH_PUBLIC ROSAUTH_IMPORT
  #endif
  #define ROSAUTH_PUBLIC_TYPE ROSAUTH_PUBLIC
  #define ROSAUTH_LOCAL
#else
  #define ROSAUTH_EXPORT __attribute__ ((visibility("default")))
  #define ROSAUTH_IMPORT
  #if __GNUC__ >= 4
    #define ROSAUTH_PUBLIC __attribute__ ((visibility("default")))
    #define ROSAUTH_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROSAUTH_PUBLIC
    #define ROSAUTH_LOCAL
  #endif
  #define ROSAUTH_PUBLIC_TYPE
#endif
#endif  // ROSAUTH__VISIBILITY_CONTROL_H_
// Generated 25-Mar-2020 22:24:55
// Copyright 2019-2020 The MathWorks, Inc.
