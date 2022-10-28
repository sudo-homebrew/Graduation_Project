#ifndef WFOV_CAMERA_MSGS__VISIBILITY_CONTROL_H_
#define WFOV_CAMERA_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define WFOV_CAMERA_MSGS_EXPORT __attribute__ ((dllexport))
    #define WFOV_CAMERA_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define WFOV_CAMERA_MSGS_EXPORT __declspec(dllexport)
    #define WFOV_CAMERA_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef WFOV_CAMERA_MSGS_BUILDING_LIBRARY
    #define WFOV_CAMERA_MSGS_PUBLIC WFOV_CAMERA_MSGS_EXPORT
  #else
    #define WFOV_CAMERA_MSGS_PUBLIC WFOV_CAMERA_MSGS_IMPORT
  #endif
  #define WFOV_CAMERA_MSGS_PUBLIC_TYPE WFOV_CAMERA_MSGS_PUBLIC
  #define WFOV_CAMERA_MSGS_LOCAL
#else
  #define WFOV_CAMERA_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define WFOV_CAMERA_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define WFOV_CAMERA_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define WFOV_CAMERA_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define WFOV_CAMERA_MSGS_PUBLIC
    #define WFOV_CAMERA_MSGS_LOCAL
  #endif
  #define WFOV_CAMERA_MSGS_PUBLIC_TYPE
#endif
#endif  // WFOV_CAMERA_MSGS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:03:37
// Copyright 2019-2020 The MathWorks, Inc.
