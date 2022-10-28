#ifndef IMAGE_VIEW2__VISIBILITY_CONTROL_H_
#define IMAGE_VIEW2__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define IMAGE_VIEW2_EXPORT __attribute__ ((dllexport))
    #define IMAGE_VIEW2_IMPORT __attribute__ ((dllimport))
  #else
    #define IMAGE_VIEW2_EXPORT __declspec(dllexport)
    #define IMAGE_VIEW2_IMPORT __declspec(dllimport)
  #endif
  #ifdef IMAGE_VIEW2_BUILDING_LIBRARY
    #define IMAGE_VIEW2_PUBLIC IMAGE_VIEW2_EXPORT
  #else
    #define IMAGE_VIEW2_PUBLIC IMAGE_VIEW2_IMPORT
  #endif
  #define IMAGE_VIEW2_PUBLIC_TYPE IMAGE_VIEW2_PUBLIC
  #define IMAGE_VIEW2_LOCAL
#else
  #define IMAGE_VIEW2_EXPORT __attribute__ ((visibility("default")))
  #define IMAGE_VIEW2_IMPORT
  #if __GNUC__ >= 4
    #define IMAGE_VIEW2_PUBLIC __attribute__ ((visibility("default")))
    #define IMAGE_VIEW2_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define IMAGE_VIEW2_PUBLIC
    #define IMAGE_VIEW2_LOCAL
  #endif
  #define IMAGE_VIEW2_PUBLIC_TYPE
#endif
#endif  // IMAGE_VIEW2__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 13:51:05
// Copyright 2019-2020 The MathWorks, Inc.
