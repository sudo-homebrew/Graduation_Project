#ifndef TURTLEBOT_CALIBRATION__VISIBILITY_CONTROL_H_
#define TURTLEBOT_CALIBRATION__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TURTLEBOT_CALIBRATION_EXPORT __attribute__ ((dllexport))
    #define TURTLEBOT_CALIBRATION_IMPORT __attribute__ ((dllimport))
  #else
    #define TURTLEBOT_CALIBRATION_EXPORT __declspec(dllexport)
    #define TURTLEBOT_CALIBRATION_IMPORT __declspec(dllimport)
  #endif
  #ifdef TURTLEBOT_CALIBRATION_BUILDING_LIBRARY
    #define TURTLEBOT_CALIBRATION_PUBLIC TURTLEBOT_CALIBRATION_EXPORT
  #else
    #define TURTLEBOT_CALIBRATION_PUBLIC TURTLEBOT_CALIBRATION_IMPORT
  #endif
  #define TURTLEBOT_CALIBRATION_PUBLIC_TYPE TURTLEBOT_CALIBRATION_PUBLIC
  #define TURTLEBOT_CALIBRATION_LOCAL
#else
  #define TURTLEBOT_CALIBRATION_EXPORT __attribute__ ((visibility("default")))
  #define TURTLEBOT_CALIBRATION_IMPORT
  #if __GNUC__ >= 4
    #define TURTLEBOT_CALIBRATION_PUBLIC __attribute__ ((visibility("default")))
    #define TURTLEBOT_CALIBRATION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TURTLEBOT_CALIBRATION_PUBLIC
    #define TURTLEBOT_CALIBRATION_LOCAL
  #endif
  #define TURTLEBOT_CALIBRATION_PUBLIC_TYPE
#endif
#endif  // TURTLEBOT_CALIBRATION__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:03:13
// Copyright 2019-2020 The MathWorks, Inc.
