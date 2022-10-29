#ifndef RTT_ROS_MSGS__VISIBILITY_CONTROL_H_
#define RTT_ROS_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RTT_ROS_MSGS_EXPORT __attribute__ ((dllexport))
    #define RTT_ROS_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define RTT_ROS_MSGS_EXPORT __declspec(dllexport)
    #define RTT_ROS_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef RTT_ROS_MSGS_BUILDING_LIBRARY
    #define RTT_ROS_MSGS_PUBLIC RTT_ROS_MSGS_EXPORT
  #else
    #define RTT_ROS_MSGS_PUBLIC RTT_ROS_MSGS_IMPORT
  #endif
  #define RTT_ROS_MSGS_PUBLIC_TYPE RTT_ROS_MSGS_PUBLIC
  #define RTT_ROS_MSGS_LOCAL
#else
  #define RTT_ROS_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define RTT_ROS_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define RTT_ROS_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define RTT_ROS_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RTT_ROS_MSGS_PUBLIC
    #define RTT_ROS_MSGS_LOCAL
  #endif
  #define RTT_ROS_MSGS_PUBLIC_TYPE
#endif
#endif  // RTT_ROS_MSGS__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 14:36:14
// Copyright 2019-2020 The MathWorks, Inc.
