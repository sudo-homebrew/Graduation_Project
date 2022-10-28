#ifndef MULTISENSE_ROS__VISIBILITY_CONTROL_H_
#define MULTISENSE_ROS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MULTISENSE_ROS_EXPORT __attribute__ ((dllexport))
    #define MULTISENSE_ROS_IMPORT __attribute__ ((dllimport))
  #else
    #define MULTISENSE_ROS_EXPORT __declspec(dllexport)
    #define MULTISENSE_ROS_IMPORT __declspec(dllimport)
  #endif
  #ifdef MULTISENSE_ROS_BUILDING_LIBRARY
    #define MULTISENSE_ROS_PUBLIC MULTISENSE_ROS_EXPORT
  #else
    #define MULTISENSE_ROS_PUBLIC MULTISENSE_ROS_IMPORT
  #endif
  #define MULTISENSE_ROS_PUBLIC_TYPE MULTISENSE_ROS_PUBLIC
  #define MULTISENSE_ROS_LOCAL
#else
  #define MULTISENSE_ROS_EXPORT __attribute__ ((visibility("default")))
  #define MULTISENSE_ROS_IMPORT
  #if __GNUC__ >= 4
    #define MULTISENSE_ROS_PUBLIC __attribute__ ((visibility("default")))
    #define MULTISENSE_ROS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MULTISENSE_ROS_PUBLIC
    #define MULTISENSE_ROS_LOCAL
  #endif
  #define MULTISENSE_ROS_PUBLIC_TYPE
#endif
#endif  // MULTISENSE_ROS__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 14:26:29
// Copyright 2019-2020 The MathWorks, Inc.
