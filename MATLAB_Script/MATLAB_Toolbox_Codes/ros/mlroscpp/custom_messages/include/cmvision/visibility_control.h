#ifndef CMVISION__VISIBILITY_CONTROL_H_
#define CMVISION__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CMVISION_EXPORT __attribute__ ((dllexport))
    #define CMVISION_IMPORT __attribute__ ((dllimport))
  #else
    #define CMVISION_EXPORT __declspec(dllexport)
    #define CMVISION_IMPORT __declspec(dllimport)
  #endif
  #ifdef CMVISION_BUILDING_LIBRARY
    #define CMVISION_PUBLIC CMVISION_EXPORT
  #else
    #define CMVISION_PUBLIC CMVISION_IMPORT
  #endif
  #define CMVISION_PUBLIC_TYPE CMVISION_PUBLIC
  #define CMVISION_LOCAL
#else
  #define CMVISION_EXPORT __attribute__ ((visibility("default")))
  #define CMVISION_IMPORT
  #if __GNUC__ >= 4
    #define CMVISION_PUBLIC __attribute__ ((visibility("default")))
    #define CMVISION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CMVISION_PUBLIC
    #define CMVISION_LOCAL
  #endif
  #define CMVISION_PUBLIC_TYPE
#endif
#endif  // CMVISION__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 12:35:58
// Copyright 2019-2020 The MathWorks, Inc.
