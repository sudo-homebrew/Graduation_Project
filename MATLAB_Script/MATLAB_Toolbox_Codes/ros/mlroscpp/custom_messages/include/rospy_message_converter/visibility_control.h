#ifndef ROSPY_MESSAGE_CONVERTER__VISIBILITY_CONTROL_H_
#define ROSPY_MESSAGE_CONVERTER__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSPY_MESSAGE_CONVERTER_EXPORT __attribute__ ((dllexport))
    #define ROSPY_MESSAGE_CONVERTER_IMPORT __attribute__ ((dllimport))
  #else
    #define ROSPY_MESSAGE_CONVERTER_EXPORT __declspec(dllexport)
    #define ROSPY_MESSAGE_CONVERTER_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROSPY_MESSAGE_CONVERTER_BUILDING_LIBRARY
    #define ROSPY_MESSAGE_CONVERTER_PUBLIC ROSPY_MESSAGE_CONVERTER_EXPORT
  #else
    #define ROSPY_MESSAGE_CONVERTER_PUBLIC ROSPY_MESSAGE_CONVERTER_IMPORT
  #endif
  #define ROSPY_MESSAGE_CONVERTER_PUBLIC_TYPE ROSPY_MESSAGE_CONVERTER_PUBLIC
  #define ROSPY_MESSAGE_CONVERTER_LOCAL
#else
  #define ROSPY_MESSAGE_CONVERTER_EXPORT __attribute__ ((visibility("default")))
  #define ROSPY_MESSAGE_CONVERTER_IMPORT
  #if __GNUC__ >= 4
    #define ROSPY_MESSAGE_CONVERTER_PUBLIC __attribute__ ((visibility("default")))
    #define ROSPY_MESSAGE_CONVERTER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROSPY_MESSAGE_CONVERTER_PUBLIC
    #define ROSPY_MESSAGE_CONVERTER_LOCAL
  #endif
  #define ROSPY_MESSAGE_CONVERTER_PUBLIC_TYPE
#endif
#endif  // ROSPY_MESSAGE_CONVERTER__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 14:36:05
// Copyright 2019-2020 The MathWorks, Inc.
