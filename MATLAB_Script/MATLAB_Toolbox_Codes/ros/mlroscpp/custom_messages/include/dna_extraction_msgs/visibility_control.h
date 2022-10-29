#ifndef DNA_EXTRACTION_MSGS__VISIBILITY_CONTROL_H_
#define DNA_EXTRACTION_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DNA_EXTRACTION_MSGS_EXPORT __attribute__ ((dllexport))
    #define DNA_EXTRACTION_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define DNA_EXTRACTION_MSGS_EXPORT __declspec(dllexport)
    #define DNA_EXTRACTION_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef DNA_EXTRACTION_MSGS_BUILDING_LIBRARY
    #define DNA_EXTRACTION_MSGS_PUBLIC DNA_EXTRACTION_MSGS_EXPORT
  #else
    #define DNA_EXTRACTION_MSGS_PUBLIC DNA_EXTRACTION_MSGS_IMPORT
  #endif
  #define DNA_EXTRACTION_MSGS_PUBLIC_TYPE DNA_EXTRACTION_MSGS_PUBLIC
  #define DNA_EXTRACTION_MSGS_LOCAL
#else
  #define DNA_EXTRACTION_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define DNA_EXTRACTION_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define DNA_EXTRACTION_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define DNA_EXTRACTION_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DNA_EXTRACTION_MSGS_PUBLIC
    #define DNA_EXTRACTION_MSGS_LOCAL
  #endif
  #define DNA_EXTRACTION_MSGS_PUBLIC_TYPE
#endif
#endif  // DNA_EXTRACTION_MSGS__VISIBILITY_CONTROL_H_
// Generated 25-Mar-2020 20:24:01
// Copyright 2019-2020 The MathWorks, Inc.
