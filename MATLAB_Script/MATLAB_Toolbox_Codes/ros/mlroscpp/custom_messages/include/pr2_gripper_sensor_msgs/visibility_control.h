#ifndef PR2_GRIPPER_SENSOR_MSGS__VISIBILITY_CONTROL_H_
#define PR2_GRIPPER_SENSOR_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PR2_GRIPPER_SENSOR_MSGS_EXPORT __attribute__ ((dllexport))
    #define PR2_GRIPPER_SENSOR_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define PR2_GRIPPER_SENSOR_MSGS_EXPORT __declspec(dllexport)
    #define PR2_GRIPPER_SENSOR_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef PR2_GRIPPER_SENSOR_MSGS_BUILDING_LIBRARY
    #define PR2_GRIPPER_SENSOR_MSGS_PUBLIC PR2_GRIPPER_SENSOR_MSGS_EXPORT
  #else
    #define PR2_GRIPPER_SENSOR_MSGS_PUBLIC PR2_GRIPPER_SENSOR_MSGS_IMPORT
  #endif
  #define PR2_GRIPPER_SENSOR_MSGS_PUBLIC_TYPE PR2_GRIPPER_SENSOR_MSGS_PUBLIC
  #define PR2_GRIPPER_SENSOR_MSGS_LOCAL
#else
  #define PR2_GRIPPER_SENSOR_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define PR2_GRIPPER_SENSOR_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define PR2_GRIPPER_SENSOR_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define PR2_GRIPPER_SENSOR_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PR2_GRIPPER_SENSOR_MSGS_PUBLIC
    #define PR2_GRIPPER_SENSOR_MSGS_LOCAL
  #endif
  #define PR2_GRIPPER_SENSOR_MSGS_PUBLIC_TYPE
#endif
#endif  // PR2_GRIPPER_SENSOR_MSGS__VISIBILITY_CONTROL_H_
// Generated 11-Apr-2020 16:38:19
// Copyright 2019-2020 The MathWorks, Inc.
