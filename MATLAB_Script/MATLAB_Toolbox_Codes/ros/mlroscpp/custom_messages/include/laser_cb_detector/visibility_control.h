#ifndef LASER_CB_DETECTOR__VISIBILITY_CONTROL_H_
#define LASER_CB_DETECTOR__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define LASER_CB_DETECTOR_EXPORT __attribute__ ((dllexport))
    #define LASER_CB_DETECTOR_IMPORT __attribute__ ((dllimport))
  #else
    #define LASER_CB_DETECTOR_EXPORT __declspec(dllexport)
    #define LASER_CB_DETECTOR_IMPORT __declspec(dllimport)
  #endif
  #ifdef LASER_CB_DETECTOR_BUILDING_LIBRARY
    #define LASER_CB_DETECTOR_PUBLIC LASER_CB_DETECTOR_EXPORT
  #else
    #define LASER_CB_DETECTOR_PUBLIC LASER_CB_DETECTOR_IMPORT
  #endif
  #define LASER_CB_DETECTOR_PUBLIC_TYPE LASER_CB_DETECTOR_PUBLIC
  #define LASER_CB_DETECTOR_LOCAL
#else
  #define LASER_CB_DETECTOR_EXPORT __attribute__ ((visibility("default")))
  #define LASER_CB_DETECTOR_IMPORT
  #if __GNUC__ >= 4
    #define LASER_CB_DETECTOR_PUBLIC __attribute__ ((visibility("default")))
    #define LASER_CB_DETECTOR_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define LASER_CB_DETECTOR_PUBLIC
    #define LASER_CB_DETECTOR_LOCAL
  #endif
  #define LASER_CB_DETECTOR_PUBLIC_TYPE
#endif
#endif  // LASER_CB_DETECTOR__VISIBILITY_CONTROL_H_
// Generated 12-Apr-2020 11:46:37
// Copyright 2019-2020 The MathWorks, Inc.
