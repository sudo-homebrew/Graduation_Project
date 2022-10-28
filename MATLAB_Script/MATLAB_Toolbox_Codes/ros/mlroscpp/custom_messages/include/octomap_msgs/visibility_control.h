#ifndef OCTOMAP_MSGS__VISIBILITY_CONTROL_H_
#define OCTOMAP_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define OCTOMAP_MSGS_EXPORT __attribute__ ((dllexport))
    #define OCTOMAP_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define OCTOMAP_MSGS_EXPORT __declspec(dllexport)
    #define OCTOMAP_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef OCTOMAP_MSGS_BUILDING_LIBRARY
    #define OCTOMAP_MSGS_PUBLIC OCTOMAP_MSGS_EXPORT
  #else
    #define OCTOMAP_MSGS_PUBLIC OCTOMAP_MSGS_IMPORT
  #endif
  #define OCTOMAP_MSGS_PUBLIC_TYPE OCTOMAP_MSGS_PUBLIC
  #define OCTOMAP_MSGS_LOCAL
#else
  #define OCTOMAP_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define OCTOMAP_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define OCTOMAP_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define OCTOMAP_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define OCTOMAP_MSGS_PUBLIC
    #define OCTOMAP_MSGS_LOCAL
  #endif
  #define OCTOMAP_MSGS_PUBLIC_TYPE
#endif
#endif  // OCTOMAP_MSGS__VISIBILITY_CONTROL_H_
// Generated 12-Apr-2020 11:46:42
// Copyright 2019-2020 The MathWorks, Inc.
