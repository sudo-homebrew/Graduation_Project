#ifndef SR_UTILITIES__VISIBILITY_CONTROL_H_
#define SR_UTILITIES__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SR_UTILITIES_EXPORT __attribute__ ((dllexport))
    #define SR_UTILITIES_IMPORT __attribute__ ((dllimport))
  #else
    #define SR_UTILITIES_EXPORT __declspec(dllexport)
    #define SR_UTILITIES_IMPORT __declspec(dllimport)
  #endif
  #ifdef SR_UTILITIES_BUILDING_LIBRARY
    #define SR_UTILITIES_PUBLIC SR_UTILITIES_EXPORT
  #else
    #define SR_UTILITIES_PUBLIC SR_UTILITIES_IMPORT
  #endif
  #define SR_UTILITIES_PUBLIC_TYPE SR_UTILITIES_PUBLIC
  #define SR_UTILITIES_LOCAL
#else
  #define SR_UTILITIES_EXPORT __attribute__ ((visibility("default")))
  #define SR_UTILITIES_IMPORT
  #if __GNUC__ >= 4
    #define SR_UTILITIES_PUBLIC __attribute__ ((visibility("default")))
    #define SR_UTILITIES_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SR_UTILITIES_PUBLIC
    #define SR_UTILITIES_LOCAL
  #endif
  #define SR_UTILITIES_PUBLIC_TYPE
#endif
#endif  // SR_UTILITIES__VISIBILITY_CONTROL_H_
// Generated 06-Apr-2020 16:42:43
// Copyright 2019-2020 The MathWorks, Inc.
