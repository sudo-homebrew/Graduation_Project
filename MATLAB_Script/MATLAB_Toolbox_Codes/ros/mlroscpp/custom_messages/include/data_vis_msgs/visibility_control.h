#ifndef DATA_VIS_MSGS__VISIBILITY_CONTROL_H_
#define DATA_VIS_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DATA_VIS_MSGS_EXPORT __attribute__ ((dllexport))
    #define DATA_VIS_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define DATA_VIS_MSGS_EXPORT __declspec(dllexport)
    #define DATA_VIS_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef DATA_VIS_MSGS_BUILDING_LIBRARY
    #define DATA_VIS_MSGS_PUBLIC DATA_VIS_MSGS_EXPORT
  #else
    #define DATA_VIS_MSGS_PUBLIC DATA_VIS_MSGS_IMPORT
  #endif
  #define DATA_VIS_MSGS_PUBLIC_TYPE DATA_VIS_MSGS_PUBLIC
  #define DATA_VIS_MSGS_LOCAL
#else
  #define DATA_VIS_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define DATA_VIS_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define DATA_VIS_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define DATA_VIS_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DATA_VIS_MSGS_PUBLIC
    #define DATA_VIS_MSGS_LOCAL
  #endif
  #define DATA_VIS_MSGS_PUBLIC_TYPE
#endif
#endif  // DATA_VIS_MSGS__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 13:02:36
// Copyright 2019-2020 The MathWorks, Inc.
