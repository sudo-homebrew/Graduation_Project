#ifndef CONTROLLER_MANAGER_MSGS__VISIBILITY_CONTROL_H_
#define CONTROLLER_MANAGER_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CONTROLLER_MANAGER_MSGS_EXPORT __attribute__ ((dllexport))
    #define CONTROLLER_MANAGER_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define CONTROLLER_MANAGER_MSGS_EXPORT __declspec(dllexport)
    #define CONTROLLER_MANAGER_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef CONTROLLER_MANAGER_MSGS_BUILDING_LIBRARY
    #define CONTROLLER_MANAGER_MSGS_PUBLIC CONTROLLER_MANAGER_MSGS_EXPORT
  #else
    #define CONTROLLER_MANAGER_MSGS_PUBLIC CONTROLLER_MANAGER_MSGS_IMPORT
  #endif
  #define CONTROLLER_MANAGER_MSGS_PUBLIC_TYPE CONTROLLER_MANAGER_MSGS_PUBLIC
  #define CONTROLLER_MANAGER_MSGS_LOCAL
#else
  #define CONTROLLER_MANAGER_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define CONTROLLER_MANAGER_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define CONTROLLER_MANAGER_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define CONTROLLER_MANAGER_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CONTROLLER_MANAGER_MSGS_PUBLIC
    #define CONTROLLER_MANAGER_MSGS_LOCAL
  #endif
  #define CONTROLLER_MANAGER_MSGS_PUBLIC_TYPE
#endif
#endif  // CONTROLLER_MANAGER_MSGS__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 13:02:31
// Copyright 2019-2020 The MathWorks, Inc.
