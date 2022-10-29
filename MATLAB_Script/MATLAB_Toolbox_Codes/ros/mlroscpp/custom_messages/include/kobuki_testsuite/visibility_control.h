#ifndef KOBUKI_TESTSUITE__VISIBILITY_CONTROL_H_
#define KOBUKI_TESTSUITE__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define KOBUKI_TESTSUITE_EXPORT __attribute__ ((dllexport))
    #define KOBUKI_TESTSUITE_IMPORT __attribute__ ((dllimport))
  #else
    #define KOBUKI_TESTSUITE_EXPORT __declspec(dllexport)
    #define KOBUKI_TESTSUITE_IMPORT __declspec(dllimport)
  #endif
  #ifdef KOBUKI_TESTSUITE_BUILDING_LIBRARY
    #define KOBUKI_TESTSUITE_PUBLIC KOBUKI_TESTSUITE_EXPORT
  #else
    #define KOBUKI_TESTSUITE_PUBLIC KOBUKI_TESTSUITE_IMPORT
  #endif
  #define KOBUKI_TESTSUITE_PUBLIC_TYPE KOBUKI_TESTSUITE_PUBLIC
  #define KOBUKI_TESTSUITE_LOCAL
#else
  #define KOBUKI_TESTSUITE_EXPORT __attribute__ ((visibility("default")))
  #define KOBUKI_TESTSUITE_IMPORT
  #if __GNUC__ >= 4
    #define KOBUKI_TESTSUITE_PUBLIC __attribute__ ((visibility("default")))
    #define KOBUKI_TESTSUITE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define KOBUKI_TESTSUITE_PUBLIC
    #define KOBUKI_TESTSUITE_LOCAL
  #endif
  #define KOBUKI_TESTSUITE_PUBLIC_TYPE
#endif
#endif  // KOBUKI_TESTSUITE__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:00:37
// Copyright 2019-2020 The MathWorks, Inc.
