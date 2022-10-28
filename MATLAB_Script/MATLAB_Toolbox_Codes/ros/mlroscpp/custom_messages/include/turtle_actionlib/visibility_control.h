#ifndef TURTLE_ACTIONLIB__VISIBILITY_CONTROL_H_
#define TURTLE_ACTIONLIB__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TURTLE_ACTIONLIB_EXPORT __attribute__ ((dllexport))
    #define TURTLE_ACTIONLIB_IMPORT __attribute__ ((dllimport))
  #else
    #define TURTLE_ACTIONLIB_EXPORT __declspec(dllexport)
    #define TURTLE_ACTIONLIB_IMPORT __declspec(dllimport)
  #endif
  #ifdef TURTLE_ACTIONLIB_BUILDING_LIBRARY
    #define TURTLE_ACTIONLIB_PUBLIC TURTLE_ACTIONLIB_EXPORT
  #else
    #define TURTLE_ACTIONLIB_PUBLIC TURTLE_ACTIONLIB_IMPORT
  #endif
  #define TURTLE_ACTIONLIB_PUBLIC_TYPE TURTLE_ACTIONLIB_PUBLIC
  #define TURTLE_ACTIONLIB_LOCAL
#else
  #define TURTLE_ACTIONLIB_EXPORT __attribute__ ((visibility("default")))
  #define TURTLE_ACTIONLIB_IMPORT
  #if __GNUC__ >= 4
    #define TURTLE_ACTIONLIB_PUBLIC __attribute__ ((visibility("default")))
    #define TURTLE_ACTIONLIB_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TURTLE_ACTIONLIB_PUBLIC
    #define TURTLE_ACTIONLIB_LOCAL
  #endif
  #define TURTLE_ACTIONLIB_PUBLIC_TYPE
#endif
#endif  // TURTLE_ACTIONLIB__VISIBILITY_CONTROL_H_
// Generated 11-Apr-2020 16:38:22
// Copyright 2019-2020 The MathWorks, Inc.
