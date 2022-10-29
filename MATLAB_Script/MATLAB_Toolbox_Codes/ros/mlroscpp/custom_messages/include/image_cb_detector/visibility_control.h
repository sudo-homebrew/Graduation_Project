#ifndef IMAGE_CB_DETECTOR__VISIBILITY_CONTROL_H_
#define IMAGE_CB_DETECTOR__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define IMAGE_CB_DETECTOR_EXPORT __attribute__ ((dllexport))
    #define IMAGE_CB_DETECTOR_IMPORT __attribute__ ((dllimport))
  #else
    #define IMAGE_CB_DETECTOR_EXPORT __declspec(dllexport)
    #define IMAGE_CB_DETECTOR_IMPORT __declspec(dllimport)
  #endif
  #ifdef IMAGE_CB_DETECTOR_BUILDING_LIBRARY
    #define IMAGE_CB_DETECTOR_PUBLIC IMAGE_CB_DETECTOR_EXPORT
  #else
    #define IMAGE_CB_DETECTOR_PUBLIC IMAGE_CB_DETECTOR_IMPORT
  #endif
  #define IMAGE_CB_DETECTOR_PUBLIC_TYPE IMAGE_CB_DETECTOR_PUBLIC
  #define IMAGE_CB_DETECTOR_LOCAL
#else
  #define IMAGE_CB_DETECTOR_EXPORT __attribute__ ((visibility("default")))
  #define IMAGE_CB_DETECTOR_IMPORT
  #if __GNUC__ >= 4
    #define IMAGE_CB_DETECTOR_PUBLIC __attribute__ ((visibility("default")))
    #define IMAGE_CB_DETECTOR_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define IMAGE_CB_DETECTOR_PUBLIC
    #define IMAGE_CB_DETECTOR_LOCAL
  #endif
  #define IMAGE_CB_DETECTOR_PUBLIC_TYPE
#endif
#endif  // IMAGE_CB_DETECTOR__VISIBILITY_CONTROL_H_
// Generated 12-Apr-2020 11:46:34
// Copyright 2019-2020 The MathWorks, Inc.
