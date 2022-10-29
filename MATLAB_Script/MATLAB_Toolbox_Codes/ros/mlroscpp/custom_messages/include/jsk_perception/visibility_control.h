#ifndef JSK_PERCEPTION__VISIBILITY_CONTROL_H_
#define JSK_PERCEPTION__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define JSK_PERCEPTION_EXPORT __attribute__ ((dllexport))
    #define JSK_PERCEPTION_IMPORT __attribute__ ((dllimport))
  #else
    #define JSK_PERCEPTION_EXPORT __declspec(dllexport)
    #define JSK_PERCEPTION_IMPORT __declspec(dllimport)
  #endif
  #ifdef JSK_PERCEPTION_BUILDING_LIBRARY
    #define JSK_PERCEPTION_PUBLIC JSK_PERCEPTION_EXPORT
  #else
    #define JSK_PERCEPTION_PUBLIC JSK_PERCEPTION_IMPORT
  #endif
  #define JSK_PERCEPTION_PUBLIC_TYPE JSK_PERCEPTION_PUBLIC
  #define JSK_PERCEPTION_LOCAL
#else
  #define JSK_PERCEPTION_EXPORT __attribute__ ((visibility("default")))
  #define JSK_PERCEPTION_IMPORT
  #if __GNUC__ >= 4
    #define JSK_PERCEPTION_PUBLIC __attribute__ ((visibility("default")))
    #define JSK_PERCEPTION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define JSK_PERCEPTION_PUBLIC
    #define JSK_PERCEPTION_LOCAL
  #endif
  #define JSK_PERCEPTION_PUBLIC_TYPE
#endif
#endif  // JSK_PERCEPTION__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:00:24
// Copyright 2019-2020 The MathWorks, Inc.
