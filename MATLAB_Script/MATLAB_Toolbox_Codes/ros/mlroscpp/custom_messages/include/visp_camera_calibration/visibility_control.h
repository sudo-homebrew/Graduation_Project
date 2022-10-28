#ifndef VISP_CAMERA_CALIBRATION__VISIBILITY_CONTROL_H_
#define VISP_CAMERA_CALIBRATION__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define VISP_CAMERA_CALIBRATION_EXPORT __attribute__ ((dllexport))
    #define VISP_CAMERA_CALIBRATION_IMPORT __attribute__ ((dllimport))
  #else
    #define VISP_CAMERA_CALIBRATION_EXPORT __declspec(dllexport)
    #define VISP_CAMERA_CALIBRATION_IMPORT __declspec(dllimport)
  #endif
  #ifdef VISP_CAMERA_CALIBRATION_BUILDING_LIBRARY
    #define VISP_CAMERA_CALIBRATION_PUBLIC VISP_CAMERA_CALIBRATION_EXPORT
  #else
    #define VISP_CAMERA_CALIBRATION_PUBLIC VISP_CAMERA_CALIBRATION_IMPORT
  #endif
  #define VISP_CAMERA_CALIBRATION_PUBLIC_TYPE VISP_CAMERA_CALIBRATION_PUBLIC
  #define VISP_CAMERA_CALIBRATION_LOCAL
#else
  #define VISP_CAMERA_CALIBRATION_EXPORT __attribute__ ((visibility("default")))
  #define VISP_CAMERA_CALIBRATION_IMPORT
  #if __GNUC__ >= 4
    #define VISP_CAMERA_CALIBRATION_PUBLIC __attribute__ ((visibility("default")))
    #define VISP_CAMERA_CALIBRATION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define VISP_CAMERA_CALIBRATION_PUBLIC
    #define VISP_CAMERA_CALIBRATION_LOCAL
  #endif
  #define VISP_CAMERA_CALIBRATION_PUBLIC_TYPE
#endif
#endif  // VISP_CAMERA_CALIBRATION__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:03:29
// Copyright 2019-2020 The MathWorks, Inc.
