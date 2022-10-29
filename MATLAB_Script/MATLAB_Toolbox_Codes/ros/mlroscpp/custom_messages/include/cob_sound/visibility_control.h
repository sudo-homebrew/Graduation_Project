#ifndef COB_SOUND__VISIBILITY_CONTROL_H_
#define COB_SOUND__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define COB_SOUND_EXPORT __attribute__ ((dllexport))
    #define COB_SOUND_IMPORT __attribute__ ((dllimport))
  #else
    #define COB_SOUND_EXPORT __declspec(dllexport)
    #define COB_SOUND_IMPORT __declspec(dllimport)
  #endif
  #ifdef COB_SOUND_BUILDING_LIBRARY
    #define COB_SOUND_PUBLIC COB_SOUND_EXPORT
  #else
    #define COB_SOUND_PUBLIC COB_SOUND_IMPORT
  #endif
  #define COB_SOUND_PUBLIC_TYPE COB_SOUND_PUBLIC
  #define COB_SOUND_LOCAL
#else
  #define COB_SOUND_EXPORT __attribute__ ((visibility("default")))
  #define COB_SOUND_IMPORT
  #if __GNUC__ >= 4
    #define COB_SOUND_PUBLIC __attribute__ ((visibility("default")))
    #define COB_SOUND_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define COB_SOUND_PUBLIC
    #define COB_SOUND_LOCAL
  #endif
  #define COB_SOUND_PUBLIC_TYPE
#endif
#endif  // COB_SOUND__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 12:37:37
// Copyright 2019-2020 The MathWorks, Inc.
