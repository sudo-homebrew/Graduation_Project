#ifndef LINUX_HARDWARE__VISIBILITY_CONTROL_H_
#define LINUX_HARDWARE__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define LINUX_HARDWARE_EXPORT __attribute__ ((dllexport))
    #define LINUX_HARDWARE_IMPORT __attribute__ ((dllimport))
  #else
    #define LINUX_HARDWARE_EXPORT __declspec(dllexport)
    #define LINUX_HARDWARE_IMPORT __declspec(dllimport)
  #endif
  #ifdef LINUX_HARDWARE_BUILDING_LIBRARY
    #define LINUX_HARDWARE_PUBLIC LINUX_HARDWARE_EXPORT
  #else
    #define LINUX_HARDWARE_PUBLIC LINUX_HARDWARE_IMPORT
  #endif
  #define LINUX_HARDWARE_PUBLIC_TYPE LINUX_HARDWARE_PUBLIC
  #define LINUX_HARDWARE_LOCAL
#else
  #define LINUX_HARDWARE_EXPORT __attribute__ ((visibility("default")))
  #define LINUX_HARDWARE_IMPORT
  #if __GNUC__ >= 4
    #define LINUX_HARDWARE_PUBLIC __attribute__ ((visibility("default")))
    #define LINUX_HARDWARE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define LINUX_HARDWARE_PUBLIC
    #define LINUX_HARDWARE_LOCAL
  #endif
  #define LINUX_HARDWARE_PUBLIC_TYPE
#endif
#endif  // LINUX_HARDWARE__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:00:41
// Copyright 2019-2020 The MathWorks, Inc.
