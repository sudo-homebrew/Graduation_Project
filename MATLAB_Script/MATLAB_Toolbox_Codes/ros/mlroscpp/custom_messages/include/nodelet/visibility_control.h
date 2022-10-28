#ifndef NODELET__VISIBILITY_CONTROL_H_
#define NODELET__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define NODELET_EXPORT __attribute__ ((dllexport))
    #define NODELET_IMPORT __attribute__ ((dllimport))
  #else
    #define NODELET_EXPORT __declspec(dllexport)
    #define NODELET_IMPORT __declspec(dllimport)
  #endif
  #ifdef NODELET_BUILDING_LIBRARY
    #define NODELET_PUBLIC NODELET_EXPORT
  #else
    #define NODELET_PUBLIC NODELET_IMPORT
  #endif
  #define NODELET_PUBLIC_TYPE NODELET_PUBLIC
  #define NODELET_LOCAL
#else
  #define NODELET_EXPORT __attribute__ ((visibility("default")))
  #define NODELET_IMPORT
  #if __GNUC__ >= 4
    #define NODELET_PUBLIC __attribute__ ((visibility("default")))
    #define NODELET_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define NODELET_PUBLIC
    #define NODELET_LOCAL
  #endif
  #define NODELET_PUBLIC_TYPE
#endif
#endif  // NODELET__VISIBILITY_CONTROL_H_
// Generated 25-Mar-2020 21:51:32
// Copyright 2019-2020 The MathWorks, Inc.
