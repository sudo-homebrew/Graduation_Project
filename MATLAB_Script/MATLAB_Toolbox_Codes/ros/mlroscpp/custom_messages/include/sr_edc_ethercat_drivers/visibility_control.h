#ifndef SR_EDC_ETHERCAT_DRIVERS__VISIBILITY_CONTROL_H_
#define SR_EDC_ETHERCAT_DRIVERS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SR_EDC_ETHERCAT_DRIVERS_EXPORT __attribute__ ((dllexport))
    #define SR_EDC_ETHERCAT_DRIVERS_IMPORT __attribute__ ((dllimport))
  #else
    #define SR_EDC_ETHERCAT_DRIVERS_EXPORT __declspec(dllexport)
    #define SR_EDC_ETHERCAT_DRIVERS_IMPORT __declspec(dllimport)
  #endif
  #ifdef SR_EDC_ETHERCAT_DRIVERS_BUILDING_LIBRARY
    #define SR_EDC_ETHERCAT_DRIVERS_PUBLIC SR_EDC_ETHERCAT_DRIVERS_EXPORT
  #else
    #define SR_EDC_ETHERCAT_DRIVERS_PUBLIC SR_EDC_ETHERCAT_DRIVERS_IMPORT
  #endif
  #define SR_EDC_ETHERCAT_DRIVERS_PUBLIC_TYPE SR_EDC_ETHERCAT_DRIVERS_PUBLIC
  #define SR_EDC_ETHERCAT_DRIVERS_LOCAL
#else
  #define SR_EDC_ETHERCAT_DRIVERS_EXPORT __attribute__ ((visibility("default")))
  #define SR_EDC_ETHERCAT_DRIVERS_IMPORT
  #if __GNUC__ >= 4
    #define SR_EDC_ETHERCAT_DRIVERS_PUBLIC __attribute__ ((visibility("default")))
    #define SR_EDC_ETHERCAT_DRIVERS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SR_EDC_ETHERCAT_DRIVERS_PUBLIC
    #define SR_EDC_ETHERCAT_DRIVERS_LOCAL
  #endif
  #define SR_EDC_ETHERCAT_DRIVERS_PUBLIC_TYPE
#endif
#endif  // SR_EDC_ETHERCAT_DRIVERS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:02:47
// Copyright 2019-2020 The MathWorks, Inc.
