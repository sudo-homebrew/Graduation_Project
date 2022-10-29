#ifndef BRIDE_TUTORIALS__VISIBILITY_CONTROL_H_
#define BRIDE_TUTORIALS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define BRIDE_TUTORIALS_EXPORT __attribute__ ((dllexport))
    #define BRIDE_TUTORIALS_IMPORT __attribute__ ((dllimport))
  #else
    #define BRIDE_TUTORIALS_EXPORT __declspec(dllexport)
    #define BRIDE_TUTORIALS_IMPORT __declspec(dllimport)
  #endif
  #ifdef BRIDE_TUTORIALS_BUILDING_LIBRARY
    #define BRIDE_TUTORIALS_PUBLIC BRIDE_TUTORIALS_EXPORT
  #else
    #define BRIDE_TUTORIALS_PUBLIC BRIDE_TUTORIALS_IMPORT
  #endif
  #define BRIDE_TUTORIALS_PUBLIC_TYPE BRIDE_TUTORIALS_PUBLIC
  #define BRIDE_TUTORIALS_LOCAL
#else
  #define BRIDE_TUTORIALS_EXPORT __attribute__ ((visibility("default")))
  #define BRIDE_TUTORIALS_IMPORT
  #if __GNUC__ >= 4
    #define BRIDE_TUTORIALS_PUBLIC __attribute__ ((visibility("default")))
    #define BRIDE_TUTORIALS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define BRIDE_TUTORIALS_PUBLIC
    #define BRIDE_TUTORIALS_LOCAL
  #endif
  #define BRIDE_TUTORIALS_PUBLIC_TYPE
#endif
#endif  // BRIDE_TUTORIALS__VISIBILITY_CONTROL_H_
// Generated 12-Apr-2020 11:46:25
// Copyright 2019-2020 The MathWorks, Inc.
