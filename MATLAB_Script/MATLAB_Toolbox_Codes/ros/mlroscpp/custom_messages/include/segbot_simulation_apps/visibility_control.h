#ifndef SEGBOT_SIMULATION_APPS__VISIBILITY_CONTROL_H_
#define SEGBOT_SIMULATION_APPS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SEGBOT_SIMULATION_APPS_EXPORT __attribute__ ((dllexport))
    #define SEGBOT_SIMULATION_APPS_IMPORT __attribute__ ((dllimport))
  #else
    #define SEGBOT_SIMULATION_APPS_EXPORT __declspec(dllexport)
    #define SEGBOT_SIMULATION_APPS_IMPORT __declspec(dllimport)
  #endif
  #ifdef SEGBOT_SIMULATION_APPS_BUILDING_LIBRARY
    #define SEGBOT_SIMULATION_APPS_PUBLIC SEGBOT_SIMULATION_APPS_EXPORT
  #else
    #define SEGBOT_SIMULATION_APPS_PUBLIC SEGBOT_SIMULATION_APPS_IMPORT
  #endif
  #define SEGBOT_SIMULATION_APPS_PUBLIC_TYPE SEGBOT_SIMULATION_APPS_PUBLIC
  #define SEGBOT_SIMULATION_APPS_LOCAL
#else
  #define SEGBOT_SIMULATION_APPS_EXPORT __attribute__ ((visibility("default")))
  #define SEGBOT_SIMULATION_APPS_IMPORT
  #if __GNUC__ >= 4
    #define SEGBOT_SIMULATION_APPS_PUBLIC __attribute__ ((visibility("default")))
    #define SEGBOT_SIMULATION_APPS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SEGBOT_SIMULATION_APPS_PUBLIC
    #define SEGBOT_SIMULATION_APPS_LOCAL
  #endif
  #define SEGBOT_SIMULATION_APPS_PUBLIC_TYPE
#endif
#endif  // SEGBOT_SIMULATION_APPS__VISIBILITY_CONTROL_H_
// Generated 25-Mar-2020 22:24:58
// Copyright 2019-2020 The MathWorks, Inc.
