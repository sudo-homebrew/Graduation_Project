#ifndef DYNAMIC_TF_PUBLISHER__VISIBILITY_CONTROL_H_
#define DYNAMIC_TF_PUBLISHER__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DYNAMIC_TF_PUBLISHER_EXPORT __attribute__ ((dllexport))
    #define DYNAMIC_TF_PUBLISHER_IMPORT __attribute__ ((dllimport))
  #else
    #define DYNAMIC_TF_PUBLISHER_EXPORT __declspec(dllexport)
    #define DYNAMIC_TF_PUBLISHER_IMPORT __declspec(dllimport)
  #endif
  #ifdef DYNAMIC_TF_PUBLISHER_BUILDING_LIBRARY
    #define DYNAMIC_TF_PUBLISHER_PUBLIC DYNAMIC_TF_PUBLISHER_EXPORT
  #else
    #define DYNAMIC_TF_PUBLISHER_PUBLIC DYNAMIC_TF_PUBLISHER_IMPORT
  #endif
  #define DYNAMIC_TF_PUBLISHER_PUBLIC_TYPE DYNAMIC_TF_PUBLISHER_PUBLIC
  #define DYNAMIC_TF_PUBLISHER_LOCAL
#else
  #define DYNAMIC_TF_PUBLISHER_EXPORT __attribute__ ((visibility("default")))
  #define DYNAMIC_TF_PUBLISHER_IMPORT
  #if __GNUC__ >= 4
    #define DYNAMIC_TF_PUBLISHER_PUBLIC __attribute__ ((visibility("default")))
    #define DYNAMIC_TF_PUBLISHER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DYNAMIC_TF_PUBLISHER_PUBLIC
    #define DYNAMIC_TF_PUBLISHER_LOCAL
  #endif
  #define DYNAMIC_TF_PUBLISHER_PUBLIC_TYPE
#endif
#endif  // DYNAMIC_TF_PUBLISHER__VISIBILITY_CONTROL_H_
// Generated 25-Mar-2020 20:24:02
// Copyright 2019-2020 The MathWorks, Inc.
