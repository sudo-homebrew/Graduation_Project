#ifndef COSTMAP_2D__VISIBILITY_CONTROL_H_
#define COSTMAP_2D__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define COSTMAP_2D_EXPORT __attribute__ ((dllexport))
    #define COSTMAP_2D_IMPORT __attribute__ ((dllimport))
  #else
    #define COSTMAP_2D_EXPORT __declspec(dllexport)
    #define COSTMAP_2D_IMPORT __declspec(dllimport)
  #endif
  #ifdef COSTMAP_2D_BUILDING_LIBRARY
    #define COSTMAP_2D_PUBLIC COSTMAP_2D_EXPORT
  #else
    #define COSTMAP_2D_PUBLIC COSTMAP_2D_IMPORT
  #endif
  #define COSTMAP_2D_PUBLIC_TYPE COSTMAP_2D_PUBLIC
  #define COSTMAP_2D_LOCAL
#else
  #define COSTMAP_2D_EXPORT __attribute__ ((visibility("default")))
  #define COSTMAP_2D_IMPORT
  #if __GNUC__ >= 4
    #define COSTMAP_2D_PUBLIC __attribute__ ((visibility("default")))
    #define COSTMAP_2D_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define COSTMAP_2D_PUBLIC
    #define COSTMAP_2D_LOCAL
  #endif
  #define COSTMAP_2D_PUBLIC_TYPE
#endif
#endif  // COSTMAP_2D__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 17:59:01
// Copyright 2019-2020 The MathWorks, Inc.
