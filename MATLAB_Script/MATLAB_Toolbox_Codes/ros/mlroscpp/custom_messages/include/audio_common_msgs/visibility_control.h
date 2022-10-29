#ifndef AUDIO_COMMON_MSGS__VISIBILITY_CONTROL_H_
#define AUDIO_COMMON_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define AUDIO_COMMON_MSGS_EXPORT __attribute__ ((dllexport))
    #define AUDIO_COMMON_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define AUDIO_COMMON_MSGS_EXPORT __declspec(dllexport)
    #define AUDIO_COMMON_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef AUDIO_COMMON_MSGS_BUILDING_LIBRARY
    #define AUDIO_COMMON_MSGS_PUBLIC AUDIO_COMMON_MSGS_EXPORT
  #else
    #define AUDIO_COMMON_MSGS_PUBLIC AUDIO_COMMON_MSGS_IMPORT
  #endif
  #define AUDIO_COMMON_MSGS_PUBLIC_TYPE AUDIO_COMMON_MSGS_PUBLIC
  #define AUDIO_COMMON_MSGS_LOCAL
#else
  #define AUDIO_COMMON_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define AUDIO_COMMON_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define AUDIO_COMMON_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define AUDIO_COMMON_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define AUDIO_COMMON_MSGS_PUBLIC
    #define AUDIO_COMMON_MSGS_LOCAL
  #endif
  #define AUDIO_COMMON_MSGS_PUBLIC_TYPE
#endif
#endif  // AUDIO_COMMON_MSGS__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 12:34:39
// Copyright 2019-2020 The MathWorks, Inc.
