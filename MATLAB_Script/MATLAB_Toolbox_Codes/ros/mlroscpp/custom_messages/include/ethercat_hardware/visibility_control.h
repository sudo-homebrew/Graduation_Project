#ifndef ETHERCAT_HARDWARE__VISIBILITY_CONTROL_H_
#define ETHERCAT_HARDWARE__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ETHERCAT_HARDWARE_EXPORT __attribute__ ((dllexport))
    #define ETHERCAT_HARDWARE_IMPORT __attribute__ ((dllimport))
  #else
    #define ETHERCAT_HARDWARE_EXPORT __declspec(dllexport)
    #define ETHERCAT_HARDWARE_IMPORT __declspec(dllimport)
  #endif
  #ifdef ETHERCAT_HARDWARE_BUILDING_LIBRARY
    #define ETHERCAT_HARDWARE_PUBLIC ETHERCAT_HARDWARE_EXPORT
  #else
    #define ETHERCAT_HARDWARE_PUBLIC ETHERCAT_HARDWARE_IMPORT
  #endif
  #define ETHERCAT_HARDWARE_PUBLIC_TYPE ETHERCAT_HARDWARE_PUBLIC
  #define ETHERCAT_HARDWARE_LOCAL
#else
  #define ETHERCAT_HARDWARE_EXPORT __attribute__ ((visibility("default")))
  #define ETHERCAT_HARDWARE_IMPORT
  #if __GNUC__ >= 4
    #define ETHERCAT_HARDWARE_PUBLIC __attribute__ ((visibility("default")))
    #define ETHERCAT_HARDWARE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ETHERCAT_HARDWARE_PUBLIC
    #define ETHERCAT_HARDWARE_LOCAL
  #endif
  #define ETHERCAT_HARDWARE_PUBLIC_TYPE
#endif
#endif  // ETHERCAT_HARDWARE__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 17:59:19
// Copyright 2019-2020 The MathWorks, Inc.
