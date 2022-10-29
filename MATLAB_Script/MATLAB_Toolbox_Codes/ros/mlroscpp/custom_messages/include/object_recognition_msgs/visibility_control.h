#ifndef OBJECT_RECOGNITION_MSGS__VISIBILITY_CONTROL_H_
#define OBJECT_RECOGNITION_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define OBJECT_RECOGNITION_MSGS_EXPORT __attribute__ ((dllexport))
    #define OBJECT_RECOGNITION_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define OBJECT_RECOGNITION_MSGS_EXPORT __declspec(dllexport)
    #define OBJECT_RECOGNITION_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef OBJECT_RECOGNITION_MSGS_BUILDING_LIBRARY
    #define OBJECT_RECOGNITION_MSGS_PUBLIC OBJECT_RECOGNITION_MSGS_EXPORT
  #else
    #define OBJECT_RECOGNITION_MSGS_PUBLIC OBJECT_RECOGNITION_MSGS_IMPORT
  #endif
  #define OBJECT_RECOGNITION_MSGS_PUBLIC_TYPE OBJECT_RECOGNITION_MSGS_PUBLIC
  #define OBJECT_RECOGNITION_MSGS_LOCAL
#else
  #define OBJECT_RECOGNITION_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define OBJECT_RECOGNITION_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define OBJECT_RECOGNITION_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define OBJECT_RECOGNITION_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define OBJECT_RECOGNITION_MSGS_PUBLIC
    #define OBJECT_RECOGNITION_MSGS_LOCAL
  #endif
  #define OBJECT_RECOGNITION_MSGS_PUBLIC_TYPE
#endif
#endif  // OBJECT_RECOGNITION_MSGS__VISIBILITY_CONTROL_H_
// Generated 11-Apr-2020 16:35:14
// Copyright 2019-2020 The MathWorks, Inc.
