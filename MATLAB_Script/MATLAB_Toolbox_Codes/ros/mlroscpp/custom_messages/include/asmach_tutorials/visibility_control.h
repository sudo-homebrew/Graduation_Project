#ifndef ASMACH_TUTORIALS__VISIBILITY_CONTROL_H_
#define ASMACH_TUTORIALS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ASMACH_TUTORIALS_EXPORT __attribute__ ((dllexport))
    #define ASMACH_TUTORIALS_IMPORT __attribute__ ((dllimport))
  #else
    #define ASMACH_TUTORIALS_EXPORT __declspec(dllexport)
    #define ASMACH_TUTORIALS_IMPORT __declspec(dllimport)
  #endif
  #ifdef ASMACH_TUTORIALS_BUILDING_LIBRARY
    #define ASMACH_TUTORIALS_PUBLIC ASMACH_TUTORIALS_EXPORT
  #else
    #define ASMACH_TUTORIALS_PUBLIC ASMACH_TUTORIALS_IMPORT
  #endif
  #define ASMACH_TUTORIALS_PUBLIC_TYPE ASMACH_TUTORIALS_PUBLIC
  #define ASMACH_TUTORIALS_LOCAL
#else
  #define ASMACH_TUTORIALS_EXPORT __attribute__ ((visibility("default")))
  #define ASMACH_TUTORIALS_IMPORT
  #if __GNUC__ >= 4
    #define ASMACH_TUTORIALS_PUBLIC __attribute__ ((visibility("default")))
    #define ASMACH_TUTORIALS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ASMACH_TUTORIALS_PUBLIC
    #define ASMACH_TUTORIALS_LOCAL
  #endif
  #define ASMACH_TUTORIALS_PUBLIC_TYPE
#endif
#endif  // ASMACH_TUTORIALS__VISIBILITY_CONTROL_H_
// Generated 12-Apr-2020 11:46:24
// Copyright 2019-2020 The MathWorks, Inc.
