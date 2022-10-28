#ifndef NETWORK_MONITOR_UDP__VISIBILITY_CONTROL_H_
#define NETWORK_MONITOR_UDP__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define NETWORK_MONITOR_UDP_EXPORT __attribute__ ((dllexport))
    #define NETWORK_MONITOR_UDP_IMPORT __attribute__ ((dllimport))
  #else
    #define NETWORK_MONITOR_UDP_EXPORT __declspec(dllexport)
    #define NETWORK_MONITOR_UDP_IMPORT __declspec(dllimport)
  #endif
  #ifdef NETWORK_MONITOR_UDP_BUILDING_LIBRARY
    #define NETWORK_MONITOR_UDP_PUBLIC NETWORK_MONITOR_UDP_EXPORT
  #else
    #define NETWORK_MONITOR_UDP_PUBLIC NETWORK_MONITOR_UDP_IMPORT
  #endif
  #define NETWORK_MONITOR_UDP_PUBLIC_TYPE NETWORK_MONITOR_UDP_PUBLIC
  #define NETWORK_MONITOR_UDP_LOCAL
#else
  #define NETWORK_MONITOR_UDP_EXPORT __attribute__ ((visibility("default")))
  #define NETWORK_MONITOR_UDP_IMPORT
  #if __GNUC__ >= 4
    #define NETWORK_MONITOR_UDP_PUBLIC __attribute__ ((visibility("default")))
    #define NETWORK_MONITOR_UDP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define NETWORK_MONITOR_UDP_PUBLIC
    #define NETWORK_MONITOR_UDP_LOCAL
  #endif
  #define NETWORK_MONITOR_UDP_PUBLIC_TYPE
#endif
#endif  // NETWORK_MONITOR_UDP__VISIBILITY_CONTROL_H_
// Generated 11-Apr-2020 16:35:13
// Copyright 2019-2020 The MathWorks, Inc.
