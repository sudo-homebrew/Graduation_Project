#ifndef COB_KINEMATICS__VISIBILITY_CONTROL_H_
#define COB_KINEMATICS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define COB_KINEMATICS_EXPORT __attribute__ ((dllexport))
    #define COB_KINEMATICS_IMPORT __attribute__ ((dllimport))
  #else
    #define COB_KINEMATICS_EXPORT __declspec(dllexport)
    #define COB_KINEMATICS_IMPORT __declspec(dllimport)
  #endif
  #ifdef COB_KINEMATICS_BUILDING_LIBRARY
    #define COB_KINEMATICS_PUBLIC COB_KINEMATICS_EXPORT
  #else
    #define COB_KINEMATICS_PUBLIC COB_KINEMATICS_IMPORT
  #endif
  #define COB_KINEMATICS_PUBLIC_TYPE COB_KINEMATICS_PUBLIC
  #define COB_KINEMATICS_LOCAL
#else
  #define COB_KINEMATICS_EXPORT __attribute__ ((visibility("default")))
  #define COB_KINEMATICS_IMPORT
  #if __GNUC__ >= 4
    #define COB_KINEMATICS_PUBLIC __attribute__ ((visibility("default")))
    #define COB_KINEMATICS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define COB_KINEMATICS_PUBLIC
    #define COB_KINEMATICS_LOCAL
  #endif
  #define COB_KINEMATICS_PUBLIC_TYPE
#endif
#endif  // COB_KINEMATICS__VISIBILITY_CONTROL_H_
// Generated 26-Apr-2020 13:52:28
// Copyright 2019-2020 The MathWorks, Inc.
