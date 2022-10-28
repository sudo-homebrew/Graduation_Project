#ifndef IMAGE_EXPOSURE_MSGS__VISIBILITY_CONTROL_H_
#define IMAGE_EXPOSURE_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define IMAGE_EXPOSURE_MSGS_EXPORT __attribute__ ((dllexport))
    #define IMAGE_EXPOSURE_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define IMAGE_EXPOSURE_MSGS_EXPORT __declspec(dllexport)
    #define IMAGE_EXPOSURE_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef IMAGE_EXPOSURE_MSGS_BUILDING_LIBRARY
    #define IMAGE_EXPOSURE_MSGS_PUBLIC IMAGE_EXPOSURE_MSGS_EXPORT
  #else
    #define IMAGE_EXPOSURE_MSGS_PUBLIC IMAGE_EXPOSURE_MSGS_IMPORT
  #endif
  #define IMAGE_EXPOSURE_MSGS_PUBLIC_TYPE IMAGE_EXPOSURE_MSGS_PUBLIC
  #define IMAGE_EXPOSURE_MSGS_LOCAL
#else
  #define IMAGE_EXPOSURE_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define IMAGE_EXPOSURE_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define IMAGE_EXPOSURE_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define IMAGE_EXPOSURE_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define IMAGE_EXPOSURE_MSGS_PUBLIC
    #define IMAGE_EXPOSURE_MSGS_LOCAL
  #endif
  #define IMAGE_EXPOSURE_MSGS_PUBLIC_TYPE
#endif
#endif  // IMAGE_EXPOSURE_MSGS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:00:03
// Copyright 2019-2020 The MathWorks, Inc.
