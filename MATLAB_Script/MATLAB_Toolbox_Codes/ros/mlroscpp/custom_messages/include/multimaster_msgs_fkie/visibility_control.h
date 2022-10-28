#ifndef MULTIMASTER_MSGS_FKIE__VISIBILITY_CONTROL_H_
#define MULTIMASTER_MSGS_FKIE__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MULTIMASTER_MSGS_FKIE_EXPORT __attribute__ ((dllexport))
    #define MULTIMASTER_MSGS_FKIE_IMPORT __attribute__ ((dllimport))
  #else
    #define MULTIMASTER_MSGS_FKIE_EXPORT __declspec(dllexport)
    #define MULTIMASTER_MSGS_FKIE_IMPORT __declspec(dllimport)
  #endif
  #ifdef MULTIMASTER_MSGS_FKIE_BUILDING_LIBRARY
    #define MULTIMASTER_MSGS_FKIE_PUBLIC MULTIMASTER_MSGS_FKIE_EXPORT
  #else
    #define MULTIMASTER_MSGS_FKIE_PUBLIC MULTIMASTER_MSGS_FKIE_IMPORT
  #endif
  #define MULTIMASTER_MSGS_FKIE_PUBLIC_TYPE MULTIMASTER_MSGS_FKIE_PUBLIC
  #define MULTIMASTER_MSGS_FKIE_LOCAL
#else
  #define MULTIMASTER_MSGS_FKIE_EXPORT __attribute__ ((visibility("default")))
  #define MULTIMASTER_MSGS_FKIE_IMPORT
  #if __GNUC__ >= 4
    #define MULTIMASTER_MSGS_FKIE_PUBLIC __attribute__ ((visibility("default")))
    #define MULTIMASTER_MSGS_FKIE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MULTIMASTER_MSGS_FKIE_PUBLIC
    #define MULTIMASTER_MSGS_FKIE_LOCAL
  #endif
  #define MULTIMASTER_MSGS_FKIE_PUBLIC_TYPE
#endif
#endif  // MULTIMASTER_MSGS_FKIE__VISIBILITY_CONTROL_H_
// Generated 06-Apr-2020 20:40:49
// Copyright 2019-2020 The MathWorks, Inc.
