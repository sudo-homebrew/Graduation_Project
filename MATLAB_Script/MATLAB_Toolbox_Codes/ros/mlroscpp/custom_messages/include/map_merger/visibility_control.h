#ifndef MAP_MERGER__VISIBILITY_CONTROL_H_
#define MAP_MERGER__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MAP_MERGER_EXPORT __attribute__ ((dllexport))
    #define MAP_MERGER_IMPORT __attribute__ ((dllimport))
  #else
    #define MAP_MERGER_EXPORT __declspec(dllexport)
    #define MAP_MERGER_IMPORT __declspec(dllimport)
  #endif
  #ifdef MAP_MERGER_BUILDING_LIBRARY
    #define MAP_MERGER_PUBLIC MAP_MERGER_EXPORT
  #else
    #define MAP_MERGER_PUBLIC MAP_MERGER_IMPORT
  #endif
  #define MAP_MERGER_PUBLIC_TYPE MAP_MERGER_PUBLIC
  #define MAP_MERGER_LOCAL
#else
  #define MAP_MERGER_EXPORT __attribute__ ((visibility("default")))
  #define MAP_MERGER_IMPORT
  #if __GNUC__ >= 4
    #define MAP_MERGER_PUBLIC __attribute__ ((visibility("default")))
    #define MAP_MERGER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MAP_MERGER_PUBLIC
    #define MAP_MERGER_LOCAL
  #endif
  #define MAP_MERGER_PUBLIC_TYPE
#endif
#endif  // MAP_MERGER__VISIBILITY_CONTROL_H_
// Generated 20-Apr-2020 18:47:11
// Copyright 2019-2020 The MathWorks, Inc.
