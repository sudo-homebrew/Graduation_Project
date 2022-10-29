#ifndef SOUND_PLAY__VISIBILITY_CONTROL_H_
#define SOUND_PLAY__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SOUND_PLAY_EXPORT __attribute__ ((dllexport))
    #define SOUND_PLAY_IMPORT __attribute__ ((dllimport))
  #else
    #define SOUND_PLAY_EXPORT __declspec(dllexport)
    #define SOUND_PLAY_IMPORT __declspec(dllimport)
  #endif
  #ifdef SOUND_PLAY_BUILDING_LIBRARY
    #define SOUND_PLAY_PUBLIC SOUND_PLAY_EXPORT
  #else
    #define SOUND_PLAY_PUBLIC SOUND_PLAY_IMPORT
  #endif
  #define SOUND_PLAY_PUBLIC_TYPE SOUND_PLAY_PUBLIC
  #define SOUND_PLAY_LOCAL
#else
  #define SOUND_PLAY_EXPORT __attribute__ ((visibility("default")))
  #define SOUND_PLAY_IMPORT
  #if __GNUC__ >= 4
    #define SOUND_PLAY_PUBLIC __attribute__ ((visibility("default")))
    #define SOUND_PLAY_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SOUND_PLAY_PUBLIC
    #define SOUND_PLAY_LOCAL
  #endif
  #define SOUND_PLAY_PUBLIC_TYPE
#endif
#endif  // SOUND_PLAY__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 14:36:35
// Copyright 2019-2020 The MathWorks, Inc.
