#ifndef COB_SCRIPT_SERVER__VISIBILITY_CONTROL_H_
#define COB_SCRIPT_SERVER__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define COB_SCRIPT_SERVER_EXPORT __attribute__ ((dllexport))
    #define COB_SCRIPT_SERVER_IMPORT __attribute__ ((dllimport))
  #else
    #define COB_SCRIPT_SERVER_EXPORT __declspec(dllexport)
    #define COB_SCRIPT_SERVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef COB_SCRIPT_SERVER_BUILDING_LIBRARY
    #define COB_SCRIPT_SERVER_PUBLIC COB_SCRIPT_SERVER_EXPORT
  #else
    #define COB_SCRIPT_SERVER_PUBLIC COB_SCRIPT_SERVER_IMPORT
  #endif
  #define COB_SCRIPT_SERVER_PUBLIC_TYPE COB_SCRIPT_SERVER_PUBLIC
  #define COB_SCRIPT_SERVER_LOCAL
#else
  #define COB_SCRIPT_SERVER_EXPORT __attribute__ ((visibility("default")))
  #define COB_SCRIPT_SERVER_IMPORT
  #if __GNUC__ >= 4
    #define COB_SCRIPT_SERVER_PUBLIC __attribute__ ((visibility("default")))
    #define COB_SCRIPT_SERVER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define COB_SCRIPT_SERVER_PUBLIC
    #define COB_SCRIPT_SERVER_LOCAL
  #endif
  #define COB_SCRIPT_SERVER_PUBLIC_TYPE
#endif
#endif  // COB_SCRIPT_SERVER__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 12:37:28
// Copyright 2019-2020 The MathWorks, Inc.
