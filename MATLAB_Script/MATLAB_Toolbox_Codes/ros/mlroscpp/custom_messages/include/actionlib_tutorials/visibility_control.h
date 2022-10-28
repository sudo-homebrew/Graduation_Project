#ifndef ACTIONLIB_TUTORIALS__VISIBILITY_CONTROL_H_
#define ACTIONLIB_TUTORIALS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ACTIONLIB_TUTORIALS_EXPORT __attribute__ ((dllexport))
    #define ACTIONLIB_TUTORIALS_IMPORT __attribute__ ((dllimport))
  #else
    #define ACTIONLIB_TUTORIALS_EXPORT __declspec(dllexport)
    #define ACTIONLIB_TUTORIALS_IMPORT __declspec(dllimport)
  #endif
  #ifdef ACTIONLIB_TUTORIALS_BUILDING_LIBRARY
    #define ACTIONLIB_TUTORIALS_PUBLIC ACTIONLIB_TUTORIALS_EXPORT
  #else
    #define ACTIONLIB_TUTORIALS_PUBLIC ACTIONLIB_TUTORIALS_IMPORT
  #endif
  #define ACTIONLIB_TUTORIALS_PUBLIC_TYPE ACTIONLIB_TUTORIALS_PUBLIC
  #define ACTIONLIB_TUTORIALS_LOCAL
#else
  #define ACTIONLIB_TUTORIALS_EXPORT __attribute__ ((visibility("default")))
  #define ACTIONLIB_TUTORIALS_IMPORT
  #if __GNUC__ >= 4
    #define ACTIONLIB_TUTORIALS_PUBLIC __attribute__ ((visibility("default")))
    #define ACTIONLIB_TUTORIALS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ACTIONLIB_TUTORIALS_PUBLIC
    #define ACTIONLIB_TUTORIALS_LOCAL
  #endif
  #define ACTIONLIB_TUTORIALS_PUBLIC_TYPE
#endif
#endif  // ACTIONLIB_TUTORIALS__VISIBILITY_CONTROL_H_
// Generated 12-Apr-2020 11:46:24
// Copyright 2019-2020 The MathWorks, Inc.
