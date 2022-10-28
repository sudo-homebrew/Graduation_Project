#ifndef POSEDETECTION_MSGS__VISIBILITY_CONTROL_H_
#define POSEDETECTION_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define POSEDETECTION_MSGS_EXPORT __attribute__ ((dllexport))
    #define POSEDETECTION_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define POSEDETECTION_MSGS_EXPORT __declspec(dllexport)
    #define POSEDETECTION_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef POSEDETECTION_MSGS_BUILDING_LIBRARY
    #define POSEDETECTION_MSGS_PUBLIC POSEDETECTION_MSGS_EXPORT
  #else
    #define POSEDETECTION_MSGS_PUBLIC POSEDETECTION_MSGS_IMPORT
  #endif
  #define POSEDETECTION_MSGS_PUBLIC_TYPE POSEDETECTION_MSGS_PUBLIC
  #define POSEDETECTION_MSGS_LOCAL
#else
  #define POSEDETECTION_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define POSEDETECTION_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define POSEDETECTION_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define POSEDETECTION_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define POSEDETECTION_MSGS_PUBLIC
    #define POSEDETECTION_MSGS_LOCAL
  #endif
  #define POSEDETECTION_MSGS_PUBLIC_TYPE
#endif
#endif  // POSEDETECTION_MSGS__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 14:26:50
// Copyright 2019-2020 The MathWorks, Inc.
