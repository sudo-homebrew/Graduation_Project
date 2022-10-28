#ifndef PLAY_MOTION_MSGS__VISIBILITY_CONTROL_H_
#define PLAY_MOTION_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PLAY_MOTION_MSGS_EXPORT __attribute__ ((dllexport))
    #define PLAY_MOTION_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define PLAY_MOTION_MSGS_EXPORT __declspec(dllexport)
    #define PLAY_MOTION_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef PLAY_MOTION_MSGS_BUILDING_LIBRARY
    #define PLAY_MOTION_MSGS_PUBLIC PLAY_MOTION_MSGS_EXPORT
  #else
    #define PLAY_MOTION_MSGS_PUBLIC PLAY_MOTION_MSGS_IMPORT
  #endif
  #define PLAY_MOTION_MSGS_PUBLIC_TYPE PLAY_MOTION_MSGS_PUBLIC
  #define PLAY_MOTION_MSGS_LOCAL
#else
  #define PLAY_MOTION_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define PLAY_MOTION_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define PLAY_MOTION_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define PLAY_MOTION_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PLAY_MOTION_MSGS_PUBLIC
    #define PLAY_MOTION_MSGS_LOCAL
  #endif
  #define PLAY_MOTION_MSGS_PUBLIC_TYPE
#endif
#endif  // PLAY_MOTION_MSGS__VISIBILITY_CONTROL_H_
// Generated 11-Apr-2020 16:35:15
// Copyright 2019-2020 The MathWorks, Inc.
