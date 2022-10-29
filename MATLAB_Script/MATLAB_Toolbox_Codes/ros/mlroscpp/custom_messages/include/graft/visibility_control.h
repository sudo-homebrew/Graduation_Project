#ifndef GRAFT__VISIBILITY_CONTROL_H_
#define GRAFT__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GRAFT_EXPORT __attribute__ ((dllexport))
    #define GRAFT_IMPORT __attribute__ ((dllimport))
  #else
    #define GRAFT_EXPORT __declspec(dllexport)
    #define GRAFT_IMPORT __declspec(dllimport)
  #endif
  #ifdef GRAFT_BUILDING_LIBRARY
    #define GRAFT_PUBLIC GRAFT_EXPORT
  #else
    #define GRAFT_PUBLIC GRAFT_IMPORT
  #endif
  #define GRAFT_PUBLIC_TYPE GRAFT_PUBLIC
  #define GRAFT_LOCAL
#else
  #define GRAFT_EXPORT __attribute__ ((visibility("default")))
  #define GRAFT_IMPORT
  #if __GNUC__ >= 4
    #define GRAFT_PUBLIC __attribute__ ((visibility("default")))
    #define GRAFT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GRAFT_PUBLIC
    #define GRAFT_LOCAL
  #endif
  #define GRAFT_PUBLIC_TYPE
#endif
#endif  // GRAFT__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 17:59:40
// Copyright 2019-2020 The MathWorks, Inc.
