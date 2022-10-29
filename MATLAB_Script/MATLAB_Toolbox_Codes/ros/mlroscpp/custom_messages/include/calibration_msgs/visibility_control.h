#ifndef CALIBRATION_MSGS__VISIBILITY_CONTROL_H_
#define CALIBRATION_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CALIBRATION_MSGS_EXPORT __attribute__ ((dllexport))
    #define CALIBRATION_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define CALIBRATION_MSGS_EXPORT __declspec(dllexport)
    #define CALIBRATION_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef CALIBRATION_MSGS_BUILDING_LIBRARY
    #define CALIBRATION_MSGS_PUBLIC CALIBRATION_MSGS_EXPORT
  #else
    #define CALIBRATION_MSGS_PUBLIC CALIBRATION_MSGS_IMPORT
  #endif
  #define CALIBRATION_MSGS_PUBLIC_TYPE CALIBRATION_MSGS_PUBLIC
  #define CALIBRATION_MSGS_LOCAL
#else
  #define CALIBRATION_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define CALIBRATION_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define CALIBRATION_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define CALIBRATION_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CALIBRATION_MSGS_PUBLIC
    #define CALIBRATION_MSGS_LOCAL
  #endif
  #define CALIBRATION_MSGS_PUBLIC_TYPE
#endif
#endif  // CALIBRATION_MSGS__VISIBILITY_CONTROL_H_
// Generated 07-Apr-2020 11:51:04
// Copyright 2019-2020 The MathWorks, Inc.
