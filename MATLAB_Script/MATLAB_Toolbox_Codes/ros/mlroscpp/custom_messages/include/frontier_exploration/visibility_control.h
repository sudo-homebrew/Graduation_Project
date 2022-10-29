#ifndef FRONTIER_EXPLORATION__VISIBILITY_CONTROL_H_
#define FRONTIER_EXPLORATION__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define FRONTIER_EXPLORATION_EXPORT __attribute__ ((dllexport))
    #define FRONTIER_EXPLORATION_IMPORT __attribute__ ((dllimport))
  #else
    #define FRONTIER_EXPLORATION_EXPORT __declspec(dllexport)
    #define FRONTIER_EXPLORATION_IMPORT __declspec(dllimport)
  #endif
  #ifdef FRONTIER_EXPLORATION_BUILDING_LIBRARY
    #define FRONTIER_EXPLORATION_PUBLIC FRONTIER_EXPLORATION_EXPORT
  #else
    #define FRONTIER_EXPLORATION_PUBLIC FRONTIER_EXPLORATION_IMPORT
  #endif
  #define FRONTIER_EXPLORATION_PUBLIC_TYPE FRONTIER_EXPLORATION_PUBLIC
  #define FRONTIER_EXPLORATION_LOCAL
#else
  #define FRONTIER_EXPLORATION_EXPORT __attribute__ ((visibility("default")))
  #define FRONTIER_EXPLORATION_IMPORT
  #if __GNUC__ >= 4
    #define FRONTIER_EXPLORATION_PUBLIC __attribute__ ((visibility("default")))
    #define FRONTIER_EXPLORATION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define FRONTIER_EXPLORATION_PUBLIC
    #define FRONTIER_EXPLORATION_LOCAL
  #endif
  #define FRONTIER_EXPLORATION_PUBLIC_TYPE
#endif
#endif  // FRONTIER_EXPLORATION__VISIBILITY_CONTROL_H_
// Generated 12-Apr-2020 11:46:30
// Copyright 2019-2020 The MathWorks, Inc.
