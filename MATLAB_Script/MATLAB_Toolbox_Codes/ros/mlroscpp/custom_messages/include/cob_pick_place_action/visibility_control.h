#ifndef COB_PICK_PLACE_ACTION__VISIBILITY_CONTROL_H_
#define COB_PICK_PLACE_ACTION__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define COB_PICK_PLACE_ACTION_EXPORT __attribute__ ((dllexport))
    #define COB_PICK_PLACE_ACTION_IMPORT __attribute__ ((dllimport))
  #else
    #define COB_PICK_PLACE_ACTION_EXPORT __declspec(dllexport)
    #define COB_PICK_PLACE_ACTION_IMPORT __declspec(dllimport)
  #endif
  #ifdef COB_PICK_PLACE_ACTION_BUILDING_LIBRARY
    #define COB_PICK_PLACE_ACTION_PUBLIC COB_PICK_PLACE_ACTION_EXPORT
  #else
    #define COB_PICK_PLACE_ACTION_PUBLIC COB_PICK_PLACE_ACTION_IMPORT
  #endif
  #define COB_PICK_PLACE_ACTION_PUBLIC_TYPE COB_PICK_PLACE_ACTION_PUBLIC
  #define COB_PICK_PLACE_ACTION_LOCAL
#else
  #define COB_PICK_PLACE_ACTION_EXPORT __attribute__ ((visibility("default")))
  #define COB_PICK_PLACE_ACTION_IMPORT
  #if __GNUC__ >= 4
    #define COB_PICK_PLACE_ACTION_PUBLIC __attribute__ ((visibility("default")))
    #define COB_PICK_PLACE_ACTION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define COB_PICK_PLACE_ACTION_PUBLIC
    #define COB_PICK_PLACE_ACTION_LOCAL
  #endif
  #define COB_PICK_PLACE_ACTION_PUBLIC_TYPE
#endif
#endif  // COB_PICK_PLACE_ACTION__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 12:37:14
// Copyright 2019-2020 The MathWorks, Inc.
