#ifndef JSK_FOOTSTEP_CONTROLLER__VISIBILITY_CONTROL_H_
#define JSK_FOOTSTEP_CONTROLLER__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define JSK_FOOTSTEP_CONTROLLER_EXPORT __attribute__ ((dllexport))
    #define JSK_FOOTSTEP_CONTROLLER_IMPORT __attribute__ ((dllimport))
  #else
    #define JSK_FOOTSTEP_CONTROLLER_EXPORT __declspec(dllexport)
    #define JSK_FOOTSTEP_CONTROLLER_IMPORT __declspec(dllimport)
  #endif
  #ifdef JSK_FOOTSTEP_CONTROLLER_BUILDING_LIBRARY
    #define JSK_FOOTSTEP_CONTROLLER_PUBLIC JSK_FOOTSTEP_CONTROLLER_EXPORT
  #else
    #define JSK_FOOTSTEP_CONTROLLER_PUBLIC JSK_FOOTSTEP_CONTROLLER_IMPORT
  #endif
  #define JSK_FOOTSTEP_CONTROLLER_PUBLIC_TYPE JSK_FOOTSTEP_CONTROLLER_PUBLIC
  #define JSK_FOOTSTEP_CONTROLLER_LOCAL
#else
  #define JSK_FOOTSTEP_CONTROLLER_EXPORT __attribute__ ((visibility("default")))
  #define JSK_FOOTSTEP_CONTROLLER_IMPORT
  #if __GNUC__ >= 4
    #define JSK_FOOTSTEP_CONTROLLER_PUBLIC __attribute__ ((visibility("default")))
    #define JSK_FOOTSTEP_CONTROLLER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define JSK_FOOTSTEP_CONTROLLER_PUBLIC
    #define JSK_FOOTSTEP_CONTROLLER_LOCAL
  #endif
  #define JSK_FOOTSTEP_CONTROLLER_PUBLIC_TYPE
#endif
#endif  // JSK_FOOTSTEP_CONTROLLER__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 13:51:55
// Copyright 2019-2020 The MathWorks, Inc.
