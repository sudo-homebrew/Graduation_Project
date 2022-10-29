#ifndef JSK_PCL_ROS__VISIBILITY_CONTROL_H_
#define JSK_PCL_ROS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define JSK_PCL_ROS_EXPORT __attribute__ ((dllexport))
    #define JSK_PCL_ROS_IMPORT __attribute__ ((dllimport))
  #else
    #define JSK_PCL_ROS_EXPORT __declspec(dllexport)
    #define JSK_PCL_ROS_IMPORT __declspec(dllimport)
  #endif
  #ifdef JSK_PCL_ROS_BUILDING_LIBRARY
    #define JSK_PCL_ROS_PUBLIC JSK_PCL_ROS_EXPORT
  #else
    #define JSK_PCL_ROS_PUBLIC JSK_PCL_ROS_IMPORT
  #endif
  #define JSK_PCL_ROS_PUBLIC_TYPE JSK_PCL_ROS_PUBLIC
  #define JSK_PCL_ROS_LOCAL
#else
  #define JSK_PCL_ROS_EXPORT __attribute__ ((visibility("default")))
  #define JSK_PCL_ROS_IMPORT
  #if __GNUC__ >= 4
    #define JSK_PCL_ROS_PUBLIC __attribute__ ((visibility("default")))
    #define JSK_PCL_ROS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define JSK_PCL_ROS_PUBLIC
    #define JSK_PCL_ROS_LOCAL
  #endif
  #define JSK_PCL_ROS_PUBLIC_TYPE
#endif
#endif  // JSK_PCL_ROS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:00:22
// Copyright 2019-2020 The MathWorks, Inc.
