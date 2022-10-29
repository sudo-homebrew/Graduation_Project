#ifndef ROSRUBY_TUTORIALS__VISIBILITY_CONTROL_H_
#define ROSRUBY_TUTORIALS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSRUBY_TUTORIALS_EXPORT __attribute__ ((dllexport))
    #define ROSRUBY_TUTORIALS_IMPORT __attribute__ ((dllimport))
  #else
    #define ROSRUBY_TUTORIALS_EXPORT __declspec(dllexport)
    #define ROSRUBY_TUTORIALS_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROSRUBY_TUTORIALS_BUILDING_LIBRARY
    #define ROSRUBY_TUTORIALS_PUBLIC ROSRUBY_TUTORIALS_EXPORT
  #else
    #define ROSRUBY_TUTORIALS_PUBLIC ROSRUBY_TUTORIALS_IMPORT
  #endif
  #define ROSRUBY_TUTORIALS_PUBLIC_TYPE ROSRUBY_TUTORIALS_PUBLIC
  #define ROSRUBY_TUTORIALS_LOCAL
#else
  #define ROSRUBY_TUTORIALS_EXPORT __attribute__ ((visibility("default")))
  #define ROSRUBY_TUTORIALS_IMPORT
  #if __GNUC__ >= 4
    #define ROSRUBY_TUTORIALS_PUBLIC __attribute__ ((visibility("default")))
    #define ROSRUBY_TUTORIALS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROSRUBY_TUTORIALS_PUBLIC
    #define ROSRUBY_TUTORIALS_LOCAL
  #endif
  #define ROSRUBY_TUTORIALS_PUBLIC_TYPE
#endif
#endif  // ROSRUBY_TUTORIALS__VISIBILITY_CONTROL_H_
// Generated 25-Mar-2020 22:24:56
// Copyright 2019-2020 The MathWorks, Inc.
