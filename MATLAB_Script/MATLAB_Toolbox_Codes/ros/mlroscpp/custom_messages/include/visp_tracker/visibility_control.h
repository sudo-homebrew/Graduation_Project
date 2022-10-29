#ifndef VISP_TRACKER__VISIBILITY_CONTROL_H_
#define VISP_TRACKER__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define VISP_TRACKER_EXPORT __attribute__ ((dllexport))
    #define VISP_TRACKER_IMPORT __attribute__ ((dllimport))
  #else
    #define VISP_TRACKER_EXPORT __declspec(dllexport)
    #define VISP_TRACKER_IMPORT __declspec(dllimport)
  #endif
  #ifdef VISP_TRACKER_BUILDING_LIBRARY
    #define VISP_TRACKER_PUBLIC VISP_TRACKER_EXPORT
  #else
    #define VISP_TRACKER_PUBLIC VISP_TRACKER_IMPORT
  #endif
  #define VISP_TRACKER_PUBLIC_TYPE VISP_TRACKER_PUBLIC
  #define VISP_TRACKER_LOCAL
#else
  #define VISP_TRACKER_EXPORT __attribute__ ((visibility("default")))
  #define VISP_TRACKER_IMPORT
  #if __GNUC__ >= 4
    #define VISP_TRACKER_PUBLIC __attribute__ ((visibility("default")))
    #define VISP_TRACKER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define VISP_TRACKER_PUBLIC
    #define VISP_TRACKER_LOCAL
  #endif
  #define VISP_TRACKER_PUBLIC_TYPE
#endif
#endif  // VISP_TRACKER__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 14:37:47
// Copyright 2019-2020 The MathWorks, Inc.
