#ifndef FKIE_MULTIMASTER_MSGS__VISIBILITY_CONTROL_H_
#define FKIE_MULTIMASTER_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define FKIE_MULTIMASTER_MSGS_EXPORT __attribute__ ((dllexport))
    #define FKIE_MULTIMASTER_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define FKIE_MULTIMASTER_MSGS_EXPORT __declspec(dllexport)
    #define FKIE_MULTIMASTER_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef FKIE_MULTIMASTER_MSGS_BUILDING_LIBRARY
    #define FKIE_MULTIMASTER_MSGS_PUBLIC FKIE_MULTIMASTER_MSGS_EXPORT
  #else
    #define FKIE_MULTIMASTER_MSGS_PUBLIC FKIE_MULTIMASTER_MSGS_IMPORT
  #endif
  #define FKIE_MULTIMASTER_MSGS_PUBLIC_TYPE FKIE_MULTIMASTER_MSGS_PUBLIC
  #define FKIE_MULTIMASTER_MSGS_LOCAL
#else
  #define FKIE_MULTIMASTER_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define FKIE_MULTIMASTER_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define FKIE_MULTIMASTER_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define FKIE_MULTIMASTER_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define FKIE_MULTIMASTER_MSGS_PUBLIC
    #define FKIE_MULTIMASTER_MSGS_LOCAL
  #endif
  #define FKIE_MULTIMASTER_MSGS_PUBLIC_TYPE
#endif
#endif  // FKIE_MULTIMASTER_MSGS__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 13:02:56
// Copyright 2019-2020 The MathWorks, Inc.
