#ifndef UM6__VISIBILITY_CONTROL_H_
#define UM6__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define UM6_EXPORT __attribute__ ((dllexport))
    #define UM6_IMPORT __attribute__ ((dllimport))
  #else
    #define UM6_EXPORT __declspec(dllexport)
    #define UM6_IMPORT __declspec(dllimport)
  #endif
  #ifdef UM6_BUILDING_LIBRARY
    #define UM6_PUBLIC UM6_EXPORT
  #else
    #define UM6_PUBLIC UM6_IMPORT
  #endif
  #define UM6_PUBLIC_TYPE UM6_PUBLIC
  #define UM6_LOCAL
#else
  #define UM6_EXPORT __attribute__ ((visibility("default")))
  #define UM6_IMPORT
  #if __GNUC__ >= 4
    #define UM6_PUBLIC __attribute__ ((visibility("default")))
    #define UM6_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define UM6_PUBLIC
    #define UM6_LOCAL
  #endif
  #define UM6_PUBLIC_TYPE
#endif
#endif  // UM6__VISIBILITY_CONTROL_H_
// Generated 25-Mar-2020 22:35:27
// Copyright 2019-2020 The MathWorks, Inc.
