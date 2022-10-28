#ifndef AR_TRACK_ALVAR__VISIBILITY_CONTROL_H_
#define AR_TRACK_ALVAR__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define AR_TRACK_ALVAR_EXPORT __attribute__ ((dllexport))
    #define AR_TRACK_ALVAR_IMPORT __attribute__ ((dllimport))
  #else
    #define AR_TRACK_ALVAR_EXPORT __declspec(dllexport)
    #define AR_TRACK_ALVAR_IMPORT __declspec(dllimport)
  #endif
  #ifdef AR_TRACK_ALVAR_BUILDING_LIBRARY
    #define AR_TRACK_ALVAR_PUBLIC AR_TRACK_ALVAR_EXPORT
  #else
    #define AR_TRACK_ALVAR_PUBLIC AR_TRACK_ALVAR_IMPORT
  #endif
  #define AR_TRACK_ALVAR_PUBLIC_TYPE AR_TRACK_ALVAR_PUBLIC
  #define AR_TRACK_ALVAR_LOCAL
#else
  #define AR_TRACK_ALVAR_EXPORT __attribute__ ((visibility("default")))
  #define AR_TRACK_ALVAR_IMPORT
  #if __GNUC__ >= 4
    #define AR_TRACK_ALVAR_PUBLIC __attribute__ ((visibility("default")))
    #define AR_TRACK_ALVAR_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define AR_TRACK_ALVAR_PUBLIC
    #define AR_TRACK_ALVAR_LOCAL
  #endif
  #define AR_TRACK_ALVAR_PUBLIC_TYPE
#endif
#endif  // AR_TRACK_ALVAR__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 17:58:08
// Copyright 2019-2020 The MathWorks, Inc.
