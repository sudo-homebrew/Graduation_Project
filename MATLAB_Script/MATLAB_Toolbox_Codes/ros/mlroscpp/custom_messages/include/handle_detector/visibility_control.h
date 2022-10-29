#ifndef HANDLE_DETECTOR__VISIBILITY_CONTROL_H_
#define HANDLE_DETECTOR__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define HANDLE_DETECTOR_EXPORT __attribute__ ((dllexport))
    #define HANDLE_DETECTOR_IMPORT __attribute__ ((dllimport))
  #else
    #define HANDLE_DETECTOR_EXPORT __declspec(dllexport)
    #define HANDLE_DETECTOR_IMPORT __declspec(dllimport)
  #endif
  #ifdef HANDLE_DETECTOR_BUILDING_LIBRARY
    #define HANDLE_DETECTOR_PUBLIC HANDLE_DETECTOR_EXPORT
  #else
    #define HANDLE_DETECTOR_PUBLIC HANDLE_DETECTOR_IMPORT
  #endif
  #define HANDLE_DETECTOR_PUBLIC_TYPE HANDLE_DETECTOR_PUBLIC
  #define HANDLE_DETECTOR_LOCAL
#else
  #define HANDLE_DETECTOR_EXPORT __attribute__ ((visibility("default")))
  #define HANDLE_DETECTOR_IMPORT
  #if __GNUC__ >= 4
    #define HANDLE_DETECTOR_PUBLIC __attribute__ ((visibility("default")))
    #define HANDLE_DETECTOR_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define HANDLE_DETECTOR_PUBLIC
    #define HANDLE_DETECTOR_LOCAL
  #endif
  #define HANDLE_DETECTOR_PUBLIC_TYPE
#endif
#endif  // HANDLE_DETECTOR__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 17:59:46
// Copyright 2019-2020 The MathWorks, Inc.
