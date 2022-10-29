#ifndef COB_FOOTPRINT_OBSERVER__VISIBILITY_CONTROL_H_
#define COB_FOOTPRINT_OBSERVER__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define COB_FOOTPRINT_OBSERVER_EXPORT __attribute__ ((dllexport))
    #define COB_FOOTPRINT_OBSERVER_IMPORT __attribute__ ((dllimport))
  #else
    #define COB_FOOTPRINT_OBSERVER_EXPORT __declspec(dllexport)
    #define COB_FOOTPRINT_OBSERVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef COB_FOOTPRINT_OBSERVER_BUILDING_LIBRARY
    #define COB_FOOTPRINT_OBSERVER_PUBLIC COB_FOOTPRINT_OBSERVER_EXPORT
  #else
    #define COB_FOOTPRINT_OBSERVER_PUBLIC COB_FOOTPRINT_OBSERVER_IMPORT
  #endif
  #define COB_FOOTPRINT_OBSERVER_PUBLIC_TYPE COB_FOOTPRINT_OBSERVER_PUBLIC
  #define COB_FOOTPRINT_OBSERVER_LOCAL
#else
  #define COB_FOOTPRINT_OBSERVER_EXPORT __attribute__ ((visibility("default")))
  #define COB_FOOTPRINT_OBSERVER_IMPORT
  #if __GNUC__ >= 4
    #define COB_FOOTPRINT_OBSERVER_PUBLIC __attribute__ ((visibility("default")))
    #define COB_FOOTPRINT_OBSERVER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define COB_FOOTPRINT_OBSERVER_PUBLIC
    #define COB_FOOTPRINT_OBSERVER_LOCAL
  #endif
  #define COB_FOOTPRINT_OBSERVER_PUBLIC_TYPE
#endif
#endif  // COB_FOOTPRINT_OBSERVER__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 12:36:00
// Copyright 2019-2020 The MathWorks, Inc.
