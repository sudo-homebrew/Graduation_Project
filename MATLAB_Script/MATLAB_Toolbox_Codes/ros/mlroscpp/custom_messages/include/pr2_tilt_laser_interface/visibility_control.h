#ifndef PR2_TILT_LASER_INTERFACE__VISIBILITY_CONTROL_H_
#define PR2_TILT_LASER_INTERFACE__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PR2_TILT_LASER_INTERFACE_EXPORT __attribute__ ((dllexport))
    #define PR2_TILT_LASER_INTERFACE_IMPORT __attribute__ ((dllimport))
  #else
    #define PR2_TILT_LASER_INTERFACE_EXPORT __declspec(dllexport)
    #define PR2_TILT_LASER_INTERFACE_IMPORT __declspec(dllimport)
  #endif
  #ifdef PR2_TILT_LASER_INTERFACE_BUILDING_LIBRARY
    #define PR2_TILT_LASER_INTERFACE_PUBLIC PR2_TILT_LASER_INTERFACE_EXPORT
  #else
    #define PR2_TILT_LASER_INTERFACE_PUBLIC PR2_TILT_LASER_INTERFACE_IMPORT
  #endif
  #define PR2_TILT_LASER_INTERFACE_PUBLIC_TYPE PR2_TILT_LASER_INTERFACE_PUBLIC
  #define PR2_TILT_LASER_INTERFACE_LOCAL
#else
  #define PR2_TILT_LASER_INTERFACE_EXPORT __attribute__ ((visibility("default")))
  #define PR2_TILT_LASER_INTERFACE_IMPORT
  #if __GNUC__ >= 4
    #define PR2_TILT_LASER_INTERFACE_PUBLIC __attribute__ ((visibility("default")))
    #define PR2_TILT_LASER_INTERFACE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PR2_TILT_LASER_INTERFACE_PUBLIC
    #define PR2_TILT_LASER_INTERFACE_LOCAL
  #endif
  #define PR2_TILT_LASER_INTERFACE_PUBLIC_TYPE
#endif
#endif  // PR2_TILT_LASER_INTERFACE__VISIBILITY_CONTROL_H_
// Generated 11-Apr-2020 16:38:21
// Copyright 2019-2020 The MathWorks, Inc.
