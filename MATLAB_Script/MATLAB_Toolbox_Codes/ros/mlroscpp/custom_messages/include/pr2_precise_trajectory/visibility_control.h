#ifndef PR2_PRECISE_TRAJECTORY__VISIBILITY_CONTROL_H_
#define PR2_PRECISE_TRAJECTORY__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PR2_PRECISE_TRAJECTORY_EXPORT __attribute__ ((dllexport))
    #define PR2_PRECISE_TRAJECTORY_IMPORT __attribute__ ((dllimport))
  #else
    #define PR2_PRECISE_TRAJECTORY_EXPORT __declspec(dllexport)
    #define PR2_PRECISE_TRAJECTORY_IMPORT __declspec(dllimport)
  #endif
  #ifdef PR2_PRECISE_TRAJECTORY_BUILDING_LIBRARY
    #define PR2_PRECISE_TRAJECTORY_PUBLIC PR2_PRECISE_TRAJECTORY_EXPORT
  #else
    #define PR2_PRECISE_TRAJECTORY_PUBLIC PR2_PRECISE_TRAJECTORY_IMPORT
  #endif
  #define PR2_PRECISE_TRAJECTORY_PUBLIC_TYPE PR2_PRECISE_TRAJECTORY_PUBLIC
  #define PR2_PRECISE_TRAJECTORY_LOCAL
#else
  #define PR2_PRECISE_TRAJECTORY_EXPORT __attribute__ ((visibility("default")))
  #define PR2_PRECISE_TRAJECTORY_IMPORT
  #if __GNUC__ >= 4
    #define PR2_PRECISE_TRAJECTORY_PUBLIC __attribute__ ((visibility("default")))
    #define PR2_PRECISE_TRAJECTORY_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PR2_PRECISE_TRAJECTORY_PUBLIC
    #define PR2_PRECISE_TRAJECTORY_LOCAL
  #endif
  #define PR2_PRECISE_TRAJECTORY_PUBLIC_TYPE
#endif
#endif  // PR2_PRECISE_TRAJECTORY__VISIBILITY_CONTROL_H_
// Generated 11-Apr-2020 16:38:20
// Copyright 2019-2020 The MathWorks, Inc.
