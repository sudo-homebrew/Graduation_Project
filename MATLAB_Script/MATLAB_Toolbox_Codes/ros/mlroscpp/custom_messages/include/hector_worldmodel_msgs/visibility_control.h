#ifndef HECTOR_WORLDMODEL_MSGS__VISIBILITY_CONTROL_H_
#define HECTOR_WORLDMODEL_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define HECTOR_WORLDMODEL_MSGS_EXPORT __attribute__ ((dllexport))
    #define HECTOR_WORLDMODEL_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define HECTOR_WORLDMODEL_MSGS_EXPORT __declspec(dllexport)
    #define HECTOR_WORLDMODEL_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef HECTOR_WORLDMODEL_MSGS_BUILDING_LIBRARY
    #define HECTOR_WORLDMODEL_MSGS_PUBLIC HECTOR_WORLDMODEL_MSGS_EXPORT
  #else
    #define HECTOR_WORLDMODEL_MSGS_PUBLIC HECTOR_WORLDMODEL_MSGS_IMPORT
  #endif
  #define HECTOR_WORLDMODEL_MSGS_PUBLIC_TYPE HECTOR_WORLDMODEL_MSGS_PUBLIC
  #define HECTOR_WORLDMODEL_MSGS_LOCAL
#else
  #define HECTOR_WORLDMODEL_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define HECTOR_WORLDMODEL_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define HECTOR_WORLDMODEL_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define HECTOR_WORLDMODEL_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define HECTOR_WORLDMODEL_MSGS_PUBLIC
    #define HECTOR_WORLDMODEL_MSGS_LOCAL
  #endif
  #define HECTOR_WORLDMODEL_MSGS_PUBLIC_TYPE
#endif
#endif  // HECTOR_WORLDMODEL_MSGS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 17:59:53
// Copyright 2019-2020 The MathWorks, Inc.
