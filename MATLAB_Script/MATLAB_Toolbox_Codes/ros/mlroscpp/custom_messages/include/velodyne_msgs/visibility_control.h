#ifndef VELODYNE_MSGS__VISIBILITY_CONTROL_H_
#define VELODYNE_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define VELODYNE_MSGS_EXPORT __attribute__ ((dllexport))
    #define VELODYNE_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define VELODYNE_MSGS_EXPORT __declspec(dllexport)
    #define VELODYNE_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef VELODYNE_MSGS_BUILDING_LIBRARY
    #define VELODYNE_MSGS_PUBLIC VELODYNE_MSGS_EXPORT
  #else
    #define VELODYNE_MSGS_PUBLIC VELODYNE_MSGS_IMPORT
  #endif
  #define VELODYNE_MSGS_PUBLIC_TYPE VELODYNE_MSGS_PUBLIC
  #define VELODYNE_MSGS_LOCAL
#else
  #define VELODYNE_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define VELODYNE_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define VELODYNE_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define VELODYNE_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define VELODYNE_MSGS_PUBLIC
    #define VELODYNE_MSGS_LOCAL
  #endif
  #define VELODYNE_MSGS_PUBLIC_TYPE
#endif
#endif  // VELODYNE_MSGS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:03:25
// Copyright 2019-2020 The MathWorks, Inc.
