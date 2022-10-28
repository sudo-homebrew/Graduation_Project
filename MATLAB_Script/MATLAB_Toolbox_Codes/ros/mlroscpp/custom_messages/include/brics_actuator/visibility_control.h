#ifndef BRICS_ACTUATOR__VISIBILITY_CONTROL_H_
#define BRICS_ACTUATOR__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define BRICS_ACTUATOR_EXPORT __attribute__ ((dllexport))
    #define BRICS_ACTUATOR_IMPORT __attribute__ ((dllimport))
  #else
    #define BRICS_ACTUATOR_EXPORT __declspec(dllexport)
    #define BRICS_ACTUATOR_IMPORT __declspec(dllimport)
  #endif
  #ifdef BRICS_ACTUATOR_BUILDING_LIBRARY
    #define BRICS_ACTUATOR_PUBLIC BRICS_ACTUATOR_EXPORT
  #else
    #define BRICS_ACTUATOR_PUBLIC BRICS_ACTUATOR_IMPORT
  #endif
  #define BRICS_ACTUATOR_PUBLIC_TYPE BRICS_ACTUATOR_PUBLIC
  #define BRICS_ACTUATOR_LOCAL
#else
  #define BRICS_ACTUATOR_EXPORT __attribute__ ((visibility("default")))
  #define BRICS_ACTUATOR_IMPORT
  #if __GNUC__ >= 4
    #define BRICS_ACTUATOR_PUBLIC __attribute__ ((visibility("default")))
    #define BRICS_ACTUATOR_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define BRICS_ACTUATOR_PUBLIC
    #define BRICS_ACTUATOR_LOCAL
  #endif
  #define BRICS_ACTUATOR_PUBLIC_TYPE
#endif
#endif  // BRICS_ACTUATOR__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 17:58:30
// Copyright 2019-2020 The MathWorks, Inc.
