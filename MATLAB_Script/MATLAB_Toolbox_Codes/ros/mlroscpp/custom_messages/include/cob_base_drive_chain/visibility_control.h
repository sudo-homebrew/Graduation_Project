#ifndef COB_BASE_DRIVE_CHAIN__VISIBILITY_CONTROL_H_
#define COB_BASE_DRIVE_CHAIN__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define COB_BASE_DRIVE_CHAIN_EXPORT __attribute__ ((dllexport))
    #define COB_BASE_DRIVE_CHAIN_IMPORT __attribute__ ((dllimport))
  #else
    #define COB_BASE_DRIVE_CHAIN_EXPORT __declspec(dllexport)
    #define COB_BASE_DRIVE_CHAIN_IMPORT __declspec(dllimport)
  #endif
  #ifdef COB_BASE_DRIVE_CHAIN_BUILDING_LIBRARY
    #define COB_BASE_DRIVE_CHAIN_PUBLIC COB_BASE_DRIVE_CHAIN_EXPORT
  #else
    #define COB_BASE_DRIVE_CHAIN_PUBLIC COB_BASE_DRIVE_CHAIN_IMPORT
  #endif
  #define COB_BASE_DRIVE_CHAIN_PUBLIC_TYPE COB_BASE_DRIVE_CHAIN_PUBLIC
  #define COB_BASE_DRIVE_CHAIN_LOCAL
#else
  #define COB_BASE_DRIVE_CHAIN_EXPORT __attribute__ ((visibility("default")))
  #define COB_BASE_DRIVE_CHAIN_IMPORT
  #if __GNUC__ >= 4
    #define COB_BASE_DRIVE_CHAIN_PUBLIC __attribute__ ((visibility("default")))
    #define COB_BASE_DRIVE_CHAIN_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define COB_BASE_DRIVE_CHAIN_PUBLIC
    #define COB_BASE_DRIVE_CHAIN_LOCAL
  #endif
  #define COB_BASE_DRIVE_CHAIN_PUBLIC_TYPE
#endif
#endif  // COB_BASE_DRIVE_CHAIN__VISIBILITY_CONTROL_H_
// Generated 25-Mar-2020 19:29:28
// Copyright 2019-2020 The MathWorks, Inc.
