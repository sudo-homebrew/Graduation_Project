#ifndef PCL_MSGS__VISIBILITY_CONTROL_H_
#define PCL_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PCL_MSGS_EXPORT __attribute__ ((dllexport))
    #define PCL_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define PCL_MSGS_EXPORT __declspec(dllexport)
    #define PCL_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef PCL_MSGS_BUILDING_LIBRARY
    #define PCL_MSGS_PUBLIC PCL_MSGS_EXPORT
  #else
    #define PCL_MSGS_PUBLIC PCL_MSGS_IMPORT
  #endif
  #define PCL_MSGS_PUBLIC_TYPE PCL_MSGS_PUBLIC
  #define PCL_MSGS_LOCAL
#else
  #define PCL_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define PCL_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define PCL_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define PCL_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PCL_MSGS_PUBLIC
    #define PCL_MSGS_LOCAL
  #endif
  #define PCL_MSGS_PUBLIC_TYPE
#endif
#endif  // PCL_MSGS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:01:24
// Copyright 2019-2020 The MathWorks, Inc.
