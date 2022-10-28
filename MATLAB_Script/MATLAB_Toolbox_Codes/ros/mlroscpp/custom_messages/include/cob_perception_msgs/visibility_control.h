#ifndef COB_PERCEPTION_MSGS__VISIBILITY_CONTROL_H_
#define COB_PERCEPTION_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define COB_PERCEPTION_MSGS_EXPORT __attribute__ ((dllexport))
    #define COB_PERCEPTION_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define COB_PERCEPTION_MSGS_EXPORT __declspec(dllexport)
    #define COB_PERCEPTION_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef COB_PERCEPTION_MSGS_BUILDING_LIBRARY
    #define COB_PERCEPTION_MSGS_PUBLIC COB_PERCEPTION_MSGS_EXPORT
  #else
    #define COB_PERCEPTION_MSGS_PUBLIC COB_PERCEPTION_MSGS_IMPORT
  #endif
  #define COB_PERCEPTION_MSGS_PUBLIC_TYPE COB_PERCEPTION_MSGS_PUBLIC
  #define COB_PERCEPTION_MSGS_LOCAL
#else
  #define COB_PERCEPTION_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define COB_PERCEPTION_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define COB_PERCEPTION_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define COB_PERCEPTION_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define COB_PERCEPTION_MSGS_PUBLIC
    #define COB_PERCEPTION_MSGS_LOCAL
  #endif
  #define COB_PERCEPTION_MSGS_PUBLIC_TYPE
#endif
#endif  // COB_PERCEPTION_MSGS__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 12:36:55
// Copyright 2019-2020 The MathWorks, Inc.
