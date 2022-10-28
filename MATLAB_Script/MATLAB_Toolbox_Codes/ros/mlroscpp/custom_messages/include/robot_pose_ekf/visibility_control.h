#ifndef ROBOT_POSE_EKF__VISIBILITY_CONTROL_H_
#define ROBOT_POSE_EKF__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROBOT_POSE_EKF_EXPORT __attribute__ ((dllexport))
    #define ROBOT_POSE_EKF_IMPORT __attribute__ ((dllimport))
  #else
    #define ROBOT_POSE_EKF_EXPORT __declspec(dllexport)
    #define ROBOT_POSE_EKF_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROBOT_POSE_EKF_BUILDING_LIBRARY
    #define ROBOT_POSE_EKF_PUBLIC ROBOT_POSE_EKF_EXPORT
  #else
    #define ROBOT_POSE_EKF_PUBLIC ROBOT_POSE_EKF_IMPORT
  #endif
  #define ROBOT_POSE_EKF_PUBLIC_TYPE ROBOT_POSE_EKF_PUBLIC
  #define ROBOT_POSE_EKF_LOCAL
#else
  #define ROBOT_POSE_EKF_EXPORT __attribute__ ((visibility("default")))
  #define ROBOT_POSE_EKF_IMPORT
  #if __GNUC__ >= 4
    #define ROBOT_POSE_EKF_PUBLIC __attribute__ ((visibility("default")))
    #define ROBOT_POSE_EKF_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROBOT_POSE_EKF_PUBLIC
    #define ROBOT_POSE_EKF_LOCAL
  #endif
  #define ROBOT_POSE_EKF_PUBLIC_TYPE
#endif
#endif  // ROBOT_POSE_EKF__VISIBILITY_CONTROL_H_
// Generated 25-Mar-2020 22:24:55
// Copyright 2019-2020 The MathWorks, Inc.
