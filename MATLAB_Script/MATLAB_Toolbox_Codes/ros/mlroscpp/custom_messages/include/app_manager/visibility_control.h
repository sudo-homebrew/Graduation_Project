#ifndef APP_MANAGER__VISIBILITY_CONTROL_H_
#define APP_MANAGER__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define APP_MANAGER_EXPORT __attribute__ ((dllexport))
    #define APP_MANAGER_IMPORT __attribute__ ((dllimport))
  #else
    #define APP_MANAGER_EXPORT __declspec(dllexport)
    #define APP_MANAGER_IMPORT __declspec(dllimport)
  #endif
  #ifdef APP_MANAGER_BUILDING_LIBRARY
    #define APP_MANAGER_PUBLIC APP_MANAGER_EXPORT
  #else
    #define APP_MANAGER_PUBLIC APP_MANAGER_IMPORT
  #endif
  #define APP_MANAGER_PUBLIC_TYPE APP_MANAGER_PUBLIC
  #define APP_MANAGER_LOCAL
#else
  #define APP_MANAGER_EXPORT __attribute__ ((visibility("default")))
  #define APP_MANAGER_IMPORT
  #if __GNUC__ >= 4
    #define APP_MANAGER_PUBLIC __attribute__ ((visibility("default")))
    #define APP_MANAGER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define APP_MANAGER_PUBLIC
    #define APP_MANAGER_LOCAL
  #endif
  #define APP_MANAGER_PUBLIC_TYPE
#endif
#endif  // APP_MANAGER__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 17:58:02
// Copyright 2019-2020 The MathWorks, Inc.
