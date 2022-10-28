#ifndef SCHUNK_SDH__VISIBILITY_CONTROL_H_
#define SCHUNK_SDH__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SCHUNK_SDH_EXPORT __attribute__ ((dllexport))
    #define SCHUNK_SDH_IMPORT __attribute__ ((dllimport))
  #else
    #define SCHUNK_SDH_EXPORT __declspec(dllexport)
    #define SCHUNK_SDH_IMPORT __declspec(dllimport)
  #endif
  #ifdef SCHUNK_SDH_BUILDING_LIBRARY
    #define SCHUNK_SDH_PUBLIC SCHUNK_SDH_EXPORT
  #else
    #define SCHUNK_SDH_PUBLIC SCHUNK_SDH_IMPORT
  #endif
  #define SCHUNK_SDH_PUBLIC_TYPE SCHUNK_SDH_PUBLIC
  #define SCHUNK_SDH_LOCAL
#else
  #define SCHUNK_SDH_EXPORT __attribute__ ((visibility("default")))
  #define SCHUNK_SDH_IMPORT
  #if __GNUC__ >= 4
    #define SCHUNK_SDH_PUBLIC __attribute__ ((visibility("default")))
    #define SCHUNK_SDH_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SCHUNK_SDH_PUBLIC
    #define SCHUNK_SDH_LOCAL
  #endif
  #define SCHUNK_SDH_PUBLIC_TYPE
#endif
#endif  // SCHUNK_SDH__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 14:36:25
// Copyright 2019-2020 The MathWorks, Inc.
