#ifndef VIEW_CONTROLLER_MSGS__VISIBILITY_CONTROL_H_
#define VIEW_CONTROLLER_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define VIEW_CONTROLLER_MSGS_EXPORT __attribute__ ((dllexport))
    #define VIEW_CONTROLLER_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define VIEW_CONTROLLER_MSGS_EXPORT __declspec(dllexport)
    #define VIEW_CONTROLLER_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef VIEW_CONTROLLER_MSGS_BUILDING_LIBRARY
    #define VIEW_CONTROLLER_MSGS_PUBLIC VIEW_CONTROLLER_MSGS_EXPORT
  #else
    #define VIEW_CONTROLLER_MSGS_PUBLIC VIEW_CONTROLLER_MSGS_IMPORT
  #endif
  #define VIEW_CONTROLLER_MSGS_PUBLIC_TYPE VIEW_CONTROLLER_MSGS_PUBLIC
  #define VIEW_CONTROLLER_MSGS_LOCAL
#else
  #define VIEW_CONTROLLER_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define VIEW_CONTROLLER_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define VIEW_CONTROLLER_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define VIEW_CONTROLLER_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define VIEW_CONTROLLER_MSGS_PUBLIC
    #define VIEW_CONTROLLER_MSGS_LOCAL
  #endif
  #define VIEW_CONTROLLER_MSGS_PUBLIC_TYPE
#endif
#endif  // VIEW_CONTROLLER_MSGS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:03:27
// Copyright 2019-2020 The MathWorks, Inc.
