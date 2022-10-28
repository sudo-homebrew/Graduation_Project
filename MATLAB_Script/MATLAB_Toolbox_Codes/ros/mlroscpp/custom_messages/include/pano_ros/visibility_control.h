#ifndef PANO_ROS__VISIBILITY_CONTROL_H_
#define PANO_ROS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PANO_ROS_EXPORT __attribute__ ((dllexport))
    #define PANO_ROS_IMPORT __attribute__ ((dllimport))
  #else
    #define PANO_ROS_EXPORT __declspec(dllexport)
    #define PANO_ROS_IMPORT __declspec(dllimport)
  #endif
  #ifdef PANO_ROS_BUILDING_LIBRARY
    #define PANO_ROS_PUBLIC PANO_ROS_EXPORT
  #else
    #define PANO_ROS_PUBLIC PANO_ROS_IMPORT
  #endif
  #define PANO_ROS_PUBLIC_TYPE PANO_ROS_PUBLIC
  #define PANO_ROS_LOCAL
#else
  #define PANO_ROS_EXPORT __attribute__ ((visibility("default")))
  #define PANO_ROS_IMPORT
  #if __GNUC__ >= 4
    #define PANO_ROS_PUBLIC __attribute__ ((visibility("default")))
    #define PANO_ROS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PANO_ROS_PUBLIC
    #define PANO_ROS_LOCAL
  #endif
  #define PANO_ROS_PUBLIC_TYPE
#endif
#endif  // PANO_ROS__VISIBILITY_CONTROL_H_
// Generated 11-Apr-2020 16:35:14
// Copyright 2019-2020 The MathWorks, Inc.
