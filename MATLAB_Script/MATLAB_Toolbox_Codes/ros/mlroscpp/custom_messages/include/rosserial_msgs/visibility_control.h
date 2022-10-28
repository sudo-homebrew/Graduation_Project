#ifndef ROSSERIAL_MSGS__VISIBILITY_CONTROL_H_
#define ROSSERIAL_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSSERIAL_MSGS_EXPORT __attribute__ ((dllexport))
    #define ROSSERIAL_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define ROSSERIAL_MSGS_EXPORT __declspec(dllexport)
    #define ROSSERIAL_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROSSERIAL_MSGS_BUILDING_LIBRARY
    #define ROSSERIAL_MSGS_PUBLIC ROSSERIAL_MSGS_EXPORT
  #else
    #define ROSSERIAL_MSGS_PUBLIC ROSSERIAL_MSGS_IMPORT
  #endif
  #define ROSSERIAL_MSGS_PUBLIC_TYPE ROSSERIAL_MSGS_PUBLIC
  #define ROSSERIAL_MSGS_LOCAL
#else
  #define ROSSERIAL_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define ROSSERIAL_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define ROSSERIAL_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define ROSSERIAL_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROSSERIAL_MSGS_PUBLIC
    #define ROSSERIAL_MSGS_LOCAL
  #endif
  #define ROSSERIAL_MSGS_PUBLIC_TYPE
#endif
#endif  // ROSSERIAL_MSGS__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 14:36:11
// Copyright 2019-2020 The MathWorks, Inc.
