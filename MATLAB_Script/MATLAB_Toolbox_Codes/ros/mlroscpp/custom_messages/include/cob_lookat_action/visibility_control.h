#ifndef COB_LOOKAT_ACTION__VISIBILITY_CONTROL_H_
#define COB_LOOKAT_ACTION__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define COB_LOOKAT_ACTION_EXPORT __attribute__ ((dllexport))
    #define COB_LOOKAT_ACTION_IMPORT __attribute__ ((dllimport))
  #else
    #define COB_LOOKAT_ACTION_EXPORT __declspec(dllexport)
    #define COB_LOOKAT_ACTION_IMPORT __declspec(dllimport)
  #endif
  #ifdef COB_LOOKAT_ACTION_BUILDING_LIBRARY
    #define COB_LOOKAT_ACTION_PUBLIC COB_LOOKAT_ACTION_EXPORT
  #else
    #define COB_LOOKAT_ACTION_PUBLIC COB_LOOKAT_ACTION_IMPORT
  #endif
  #define COB_LOOKAT_ACTION_PUBLIC_TYPE COB_LOOKAT_ACTION_PUBLIC
  #define COB_LOOKAT_ACTION_LOCAL
#else
  #define COB_LOOKAT_ACTION_EXPORT __attribute__ ((visibility("default")))
  #define COB_LOOKAT_ACTION_IMPORT
  #if __GNUC__ >= 4
    #define COB_LOOKAT_ACTION_PUBLIC __attribute__ ((visibility("default")))
    #define COB_LOOKAT_ACTION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define COB_LOOKAT_ACTION_PUBLIC
    #define COB_LOOKAT_ACTION_LOCAL
  #endif
  #define COB_LOOKAT_ACTION_PUBLIC_TYPE
#endif
#endif  // COB_LOOKAT_ACTION__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 12:36:40
// Copyright 2019-2020 The MathWorks, Inc.
