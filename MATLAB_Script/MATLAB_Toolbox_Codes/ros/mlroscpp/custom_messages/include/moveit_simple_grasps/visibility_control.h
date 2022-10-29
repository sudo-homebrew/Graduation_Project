#ifndef MOVEIT_SIMPLE_GRASPS__VISIBILITY_CONTROL_H_
#define MOVEIT_SIMPLE_GRASPS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MOVEIT_SIMPLE_GRASPS_EXPORT __attribute__ ((dllexport))
    #define MOVEIT_SIMPLE_GRASPS_IMPORT __attribute__ ((dllimport))
  #else
    #define MOVEIT_SIMPLE_GRASPS_EXPORT __declspec(dllexport)
    #define MOVEIT_SIMPLE_GRASPS_IMPORT __declspec(dllimport)
  #endif
  #ifdef MOVEIT_SIMPLE_GRASPS_BUILDING_LIBRARY
    #define MOVEIT_SIMPLE_GRASPS_PUBLIC MOVEIT_SIMPLE_GRASPS_EXPORT
  #else
    #define MOVEIT_SIMPLE_GRASPS_PUBLIC MOVEIT_SIMPLE_GRASPS_IMPORT
  #endif
  #define MOVEIT_SIMPLE_GRASPS_PUBLIC_TYPE MOVEIT_SIMPLE_GRASPS_PUBLIC
  #define MOVEIT_SIMPLE_GRASPS_LOCAL
#else
  #define MOVEIT_SIMPLE_GRASPS_EXPORT __attribute__ ((visibility("default")))
  #define MOVEIT_SIMPLE_GRASPS_IMPORT
  #if __GNUC__ >= 4
    #define MOVEIT_SIMPLE_GRASPS_PUBLIC __attribute__ ((visibility("default")))
    #define MOVEIT_SIMPLE_GRASPS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MOVEIT_SIMPLE_GRASPS_PUBLIC
    #define MOVEIT_SIMPLE_GRASPS_LOCAL
  #endif
  #define MOVEIT_SIMPLE_GRASPS_PUBLIC_TYPE
#endif
#endif  // MOVEIT_SIMPLE_GRASPS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:00:58
// Copyright 2019-2020 The MathWorks, Inc.
