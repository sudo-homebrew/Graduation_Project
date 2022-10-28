#ifndef MONOCAM_SETTLER__VISIBILITY_CONTROL_H_
#define MONOCAM_SETTLER__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MONOCAM_SETTLER_EXPORT __attribute__ ((dllexport))
    #define MONOCAM_SETTLER_IMPORT __attribute__ ((dllimport))
  #else
    #define MONOCAM_SETTLER_EXPORT __declspec(dllexport)
    #define MONOCAM_SETTLER_IMPORT __declspec(dllimport)
  #endif
  #ifdef MONOCAM_SETTLER_BUILDING_LIBRARY
    #define MONOCAM_SETTLER_PUBLIC MONOCAM_SETTLER_EXPORT
  #else
    #define MONOCAM_SETTLER_PUBLIC MONOCAM_SETTLER_IMPORT
  #endif
  #define MONOCAM_SETTLER_PUBLIC_TYPE MONOCAM_SETTLER_PUBLIC
  #define MONOCAM_SETTLER_LOCAL
#else
  #define MONOCAM_SETTLER_EXPORT __attribute__ ((visibility("default")))
  #define MONOCAM_SETTLER_IMPORT
  #if __GNUC__ >= 4
    #define MONOCAM_SETTLER_PUBLIC __attribute__ ((visibility("default")))
    #define MONOCAM_SETTLER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MONOCAM_SETTLER_PUBLIC
    #define MONOCAM_SETTLER_LOCAL
  #endif
  #define MONOCAM_SETTLER_PUBLIC_TYPE
#endif
#endif  // MONOCAM_SETTLER__VISIBILITY_CONTROL_H_
// Generated 12-Apr-2020 11:46:39
// Copyright 2019-2020 The MathWorks, Inc.
