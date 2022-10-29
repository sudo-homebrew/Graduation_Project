#ifndef PCL_ROS__VISIBILITY_CONTROL_H_
#define PCL_ROS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PCL_ROS_EXPORT __attribute__ ((dllexport))
    #define PCL_ROS_IMPORT __attribute__ ((dllimport))
  #else
    #define PCL_ROS_EXPORT __declspec(dllexport)
    #define PCL_ROS_IMPORT __declspec(dllimport)
  #endif
  #ifdef PCL_ROS_BUILDING_LIBRARY
    #define PCL_ROS_PUBLIC PCL_ROS_EXPORT
  #else
    #define PCL_ROS_PUBLIC PCL_ROS_IMPORT
  #endif
  #define PCL_ROS_PUBLIC_TYPE PCL_ROS_PUBLIC
  #define PCL_ROS_LOCAL
#else
  #define PCL_ROS_EXPORT __attribute__ ((visibility("default")))
  #define PCL_ROS_IMPORT
  #if __GNUC__ >= 4
    #define PCL_ROS_PUBLIC __attribute__ ((visibility("default")))
    #define PCL_ROS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PCL_ROS_PUBLIC
    #define PCL_ROS_LOCAL
  #endif
  #define PCL_ROS_PUBLIC_TYPE
#endif
#endif  // PCL_ROS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:01:26
// Copyright 2019-2020 The MathWorks, Inc.
