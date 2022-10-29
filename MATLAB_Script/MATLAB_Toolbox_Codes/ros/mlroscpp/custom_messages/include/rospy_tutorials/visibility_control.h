#ifndef ROSPY_TUTORIALS__VISIBILITY_CONTROL_H_
#define ROSPY_TUTORIALS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSPY_TUTORIALS_EXPORT __attribute__ ((dllexport))
    #define ROSPY_TUTORIALS_IMPORT __attribute__ ((dllimport))
  #else
    #define ROSPY_TUTORIALS_EXPORT __declspec(dllexport)
    #define ROSPY_TUTORIALS_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROSPY_TUTORIALS_BUILDING_LIBRARY
    #define ROSPY_TUTORIALS_PUBLIC ROSPY_TUTORIALS_EXPORT
  #else
    #define ROSPY_TUTORIALS_PUBLIC ROSPY_TUTORIALS_IMPORT
  #endif
  #define ROSPY_TUTORIALS_PUBLIC_TYPE ROSPY_TUTORIALS_PUBLIC
  #define ROSPY_TUTORIALS_LOCAL
#else
  #define ROSPY_TUTORIALS_EXPORT __attribute__ ((visibility("default")))
  #define ROSPY_TUTORIALS_IMPORT
  #if __GNUC__ >= 4
    #define ROSPY_TUTORIALS_PUBLIC __attribute__ ((visibility("default")))
    #define ROSPY_TUTORIALS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROSPY_TUTORIALS_PUBLIC
    #define ROSPY_TUTORIALS_LOCAL
  #endif
  #define ROSPY_TUTORIALS_PUBLIC_TYPE
#endif
#endif  // ROSPY_TUTORIALS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:02:16
// Copyright 2019-2020 The MathWorks, Inc.
