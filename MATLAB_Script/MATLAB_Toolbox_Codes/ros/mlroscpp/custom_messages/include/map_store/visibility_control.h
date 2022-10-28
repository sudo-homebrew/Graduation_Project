#ifndef MAP_STORE__VISIBILITY_CONTROL_H_
#define MAP_STORE__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MAP_STORE_EXPORT __attribute__ ((dllexport))
    #define MAP_STORE_IMPORT __attribute__ ((dllimport))
  #else
    #define MAP_STORE_EXPORT __declspec(dllexport)
    #define MAP_STORE_IMPORT __declspec(dllimport)
  #endif
  #ifdef MAP_STORE_BUILDING_LIBRARY
    #define MAP_STORE_PUBLIC MAP_STORE_EXPORT
  #else
    #define MAP_STORE_PUBLIC MAP_STORE_IMPORT
  #endif
  #define MAP_STORE_PUBLIC_TYPE MAP_STORE_PUBLIC
  #define MAP_STORE_LOCAL
#else
  #define MAP_STORE_EXPORT __attribute__ ((visibility("default")))
  #define MAP_STORE_IMPORT
  #if __GNUC__ >= 4
    #define MAP_STORE_PUBLIC __attribute__ ((visibility("default")))
    #define MAP_STORE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MAP_STORE_PUBLIC
    #define MAP_STORE_LOCAL
  #endif
  #define MAP_STORE_PUBLIC_TYPE
#endif
#endif  // MAP_STORE__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:00:47
// Copyright 2019-2020 The MathWorks, Inc.
