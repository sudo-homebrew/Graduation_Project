#ifndef JOINT_STATES_SETTLER__VISIBILITY_CONTROL_H_
#define JOINT_STATES_SETTLER__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define JOINT_STATES_SETTLER_EXPORT __attribute__ ((dllexport))
    #define JOINT_STATES_SETTLER_IMPORT __attribute__ ((dllimport))
  #else
    #define JOINT_STATES_SETTLER_EXPORT __declspec(dllexport)
    #define JOINT_STATES_SETTLER_IMPORT __declspec(dllimport)
  #endif
  #ifdef JOINT_STATES_SETTLER_BUILDING_LIBRARY
    #define JOINT_STATES_SETTLER_PUBLIC JOINT_STATES_SETTLER_EXPORT
  #else
    #define JOINT_STATES_SETTLER_PUBLIC JOINT_STATES_SETTLER_IMPORT
  #endif
  #define JOINT_STATES_SETTLER_PUBLIC_TYPE JOINT_STATES_SETTLER_PUBLIC
  #define JOINT_STATES_SETTLER_LOCAL
#else
  #define JOINT_STATES_SETTLER_EXPORT __attribute__ ((visibility("default")))
  #define JOINT_STATES_SETTLER_IMPORT
  #if __GNUC__ >= 4
    #define JOINT_STATES_SETTLER_PUBLIC __attribute__ ((visibility("default")))
    #define JOINT_STATES_SETTLER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define JOINT_STATES_SETTLER_PUBLIC
    #define JOINT_STATES_SETTLER_LOCAL
  #endif
  #define JOINT_STATES_SETTLER_PUBLIC_TYPE
#endif
#endif  // JOINT_STATES_SETTLER__VISIBILITY_CONTROL_H_
// Generated 12-Apr-2020 11:46:35
// Copyright 2019-2020 The MathWorks, Inc.
