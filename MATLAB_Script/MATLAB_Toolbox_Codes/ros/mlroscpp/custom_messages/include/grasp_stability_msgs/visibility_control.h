#ifndef GRASP_STABILITY_MSGS__VISIBILITY_CONTROL_H_
#define GRASP_STABILITY_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GRASP_STABILITY_MSGS_EXPORT __attribute__ ((dllexport))
    #define GRASP_STABILITY_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define GRASP_STABILITY_MSGS_EXPORT __declspec(dllexport)
    #define GRASP_STABILITY_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef GRASP_STABILITY_MSGS_BUILDING_LIBRARY
    #define GRASP_STABILITY_MSGS_PUBLIC GRASP_STABILITY_MSGS_EXPORT
  #else
    #define GRASP_STABILITY_MSGS_PUBLIC GRASP_STABILITY_MSGS_IMPORT
  #endif
  #define GRASP_STABILITY_MSGS_PUBLIC_TYPE GRASP_STABILITY_MSGS_PUBLIC
  #define GRASP_STABILITY_MSGS_LOCAL
#else
  #define GRASP_STABILITY_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define GRASP_STABILITY_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define GRASP_STABILITY_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define GRASP_STABILITY_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GRASP_STABILITY_MSGS_PUBLIC
    #define GRASP_STABILITY_MSGS_LOCAL
  #endif
  #define GRASP_STABILITY_MSGS_PUBLIC_TYPE
#endif
#endif  // GRASP_STABILITY_MSGS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 17:59:44
// Copyright 2019-2020 The MathWorks, Inc.
