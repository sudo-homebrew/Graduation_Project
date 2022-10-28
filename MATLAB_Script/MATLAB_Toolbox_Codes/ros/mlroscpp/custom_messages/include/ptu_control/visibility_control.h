#ifndef PTU_CONTROL__VISIBILITY_CONTROL_H_
#define PTU_CONTROL__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PTU_CONTROL_EXPORT __attribute__ ((dllexport))
    #define PTU_CONTROL_IMPORT __attribute__ ((dllimport))
  #else
    #define PTU_CONTROL_EXPORT __declspec(dllexport)
    #define PTU_CONTROL_IMPORT __declspec(dllimport)
  #endif
  #ifdef PTU_CONTROL_BUILDING_LIBRARY
    #define PTU_CONTROL_PUBLIC PTU_CONTROL_EXPORT
  #else
    #define PTU_CONTROL_PUBLIC PTU_CONTROL_IMPORT
  #endif
  #define PTU_CONTROL_PUBLIC_TYPE PTU_CONTROL_PUBLIC
  #define PTU_CONTROL_LOCAL
#else
  #define PTU_CONTROL_EXPORT __attribute__ ((visibility("default")))
  #define PTU_CONTROL_IMPORT
  #if __GNUC__ >= 4
    #define PTU_CONTROL_PUBLIC __attribute__ ((visibility("default")))
    #define PTU_CONTROL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PTU_CONTROL_PUBLIC
    #define PTU_CONTROL_LOCAL
  #endif
  #define PTU_CONTROL_PUBLIC_TYPE
#endif
#endif  // PTU_CONTROL__VISIBILITY_CONTROL_H_
// Generated 26-Apr-2020 13:53:22
// Copyright 2019-2020 The MathWorks, Inc.
