#ifndef TF__VISIBILITY_CONTROL_H_
#define TF__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TF_EXPORT __attribute__ ((dllexport))
    #define TF_IMPORT __attribute__ ((dllimport))
  #else
    #define TF_EXPORT __declspec(dllexport)
    #define TF_IMPORT __declspec(dllimport)
  #endif
  #ifdef TF_BUILDING_LIBRARY
    #define TF_PUBLIC TF_EXPORT
  #else
    #define TF_PUBLIC TF_IMPORT
  #endif
  #define TF_PUBLIC_TYPE TF_PUBLIC
  #define TF_LOCAL
#else
  #define TF_EXPORT __attribute__ ((visibility("default")))
  #define TF_IMPORT
  #if __GNUC__ >= 4
    #define TF_PUBLIC __attribute__ ((visibility("default")))
    #define TF_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TF_PUBLIC
    #define TF_LOCAL
  #endif
  #define TF_PUBLIC_TYPE
#endif
#endif  // TF__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:03:01
// Copyright 2019-2020 The MathWorks, Inc.
