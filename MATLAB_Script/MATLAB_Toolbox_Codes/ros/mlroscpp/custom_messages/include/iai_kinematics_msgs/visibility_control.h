#ifndef IAI_KINEMATICS_MSGS__VISIBILITY_CONTROL_H_
#define IAI_KINEMATICS_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define IAI_KINEMATICS_MSGS_EXPORT __attribute__ ((dllexport))
    #define IAI_KINEMATICS_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define IAI_KINEMATICS_MSGS_EXPORT __declspec(dllexport)
    #define IAI_KINEMATICS_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef IAI_KINEMATICS_MSGS_BUILDING_LIBRARY
    #define IAI_KINEMATICS_MSGS_PUBLIC IAI_KINEMATICS_MSGS_EXPORT
  #else
    #define IAI_KINEMATICS_MSGS_PUBLIC IAI_KINEMATICS_MSGS_IMPORT
  #endif
  #define IAI_KINEMATICS_MSGS_PUBLIC_TYPE IAI_KINEMATICS_MSGS_PUBLIC
  #define IAI_KINEMATICS_MSGS_LOCAL
#else
  #define IAI_KINEMATICS_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define IAI_KINEMATICS_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define IAI_KINEMATICS_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define IAI_KINEMATICS_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define IAI_KINEMATICS_MSGS_PUBLIC
    #define IAI_KINEMATICS_MSGS_LOCAL
  #endif
  #define IAI_KINEMATICS_MSGS_PUBLIC_TYPE
#endif
#endif  // IAI_KINEMATICS_MSGS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 17:59:59
// Copyright 2019-2020 The MathWorks, Inc.
