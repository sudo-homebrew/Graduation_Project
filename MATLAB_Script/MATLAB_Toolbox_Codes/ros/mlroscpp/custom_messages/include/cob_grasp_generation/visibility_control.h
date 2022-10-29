#ifndef COB_GRASP_GENERATION__VISIBILITY_CONTROL_H_
#define COB_GRASP_GENERATION__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define COB_GRASP_GENERATION_EXPORT __attribute__ ((dllexport))
    #define COB_GRASP_GENERATION_IMPORT __attribute__ ((dllimport))
  #else
    #define COB_GRASP_GENERATION_EXPORT __declspec(dllexport)
    #define COB_GRASP_GENERATION_IMPORT __declspec(dllimport)
  #endif
  #ifdef COB_GRASP_GENERATION_BUILDING_LIBRARY
    #define COB_GRASP_GENERATION_PUBLIC COB_GRASP_GENERATION_EXPORT
  #else
    #define COB_GRASP_GENERATION_PUBLIC COB_GRASP_GENERATION_IMPORT
  #endif
  #define COB_GRASP_GENERATION_PUBLIC_TYPE COB_GRASP_GENERATION_PUBLIC
  #define COB_GRASP_GENERATION_LOCAL
#else
  #define COB_GRASP_GENERATION_EXPORT __attribute__ ((visibility("default")))
  #define COB_GRASP_GENERATION_IMPORT
  #if __GNUC__ >= 4
    #define COB_GRASP_GENERATION_PUBLIC __attribute__ ((visibility("default")))
    #define COB_GRASP_GENERATION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define COB_GRASP_GENERATION_PUBLIC
    #define COB_GRASP_GENERATION_LOCAL
  #endif
  #define COB_GRASP_GENERATION_PUBLIC_TYPE
#endif
#endif  // COB_GRASP_GENERATION__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 12:36:21
// Copyright 2019-2020 The MathWorks, Inc.
