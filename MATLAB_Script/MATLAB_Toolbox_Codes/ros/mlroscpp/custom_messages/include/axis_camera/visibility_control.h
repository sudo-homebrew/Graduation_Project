#ifndef AXIS_CAMERA__VISIBILITY_CONTROL_H_
#define AXIS_CAMERA__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define AXIS_CAMERA_EXPORT __attribute__ ((dllexport))
    #define AXIS_CAMERA_IMPORT __attribute__ ((dllimport))
  #else
    #define AXIS_CAMERA_EXPORT __declspec(dllexport)
    #define AXIS_CAMERA_IMPORT __declspec(dllimport)
  #endif
  #ifdef AXIS_CAMERA_BUILDING_LIBRARY
    #define AXIS_CAMERA_PUBLIC AXIS_CAMERA_EXPORT
  #else
    #define AXIS_CAMERA_PUBLIC AXIS_CAMERA_IMPORT
  #endif
  #define AXIS_CAMERA_PUBLIC_TYPE AXIS_CAMERA_PUBLIC
  #define AXIS_CAMERA_LOCAL
#else
  #define AXIS_CAMERA_EXPORT __attribute__ ((visibility("default")))
  #define AXIS_CAMERA_IMPORT
  #if __GNUC__ >= 4
    #define AXIS_CAMERA_PUBLIC __attribute__ ((visibility("default")))
    #define AXIS_CAMERA_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define AXIS_CAMERA_PUBLIC
    #define AXIS_CAMERA_LOCAL
  #endif
  #define AXIS_CAMERA_PUBLIC_TYPE
#endif
#endif  // AXIS_CAMERA__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 17:58:17
// Copyright 2019-2020 The MathWorks, Inc.
