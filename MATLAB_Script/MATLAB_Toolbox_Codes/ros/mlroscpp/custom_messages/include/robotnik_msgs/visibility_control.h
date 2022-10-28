#ifndef ROBOTNIK_MSGS__VISIBILITY_CONTROL_H_
#define ROBOTNIK_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROBOTNIK_MSGS_EXPORT __attribute__ ((dllexport))
    #define ROBOTNIK_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define ROBOTNIK_MSGS_EXPORT __declspec(dllexport)
    #define ROBOTNIK_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROBOTNIK_MSGS_BUILDING_LIBRARY
    #define ROBOTNIK_MSGS_PUBLIC ROBOTNIK_MSGS_EXPORT
  #else
    #define ROBOTNIK_MSGS_PUBLIC ROBOTNIK_MSGS_IMPORT
  #endif
  #define ROBOTNIK_MSGS_PUBLIC_TYPE ROBOTNIK_MSGS_PUBLIC
  #define ROBOTNIK_MSGS_LOCAL
#else
  #define ROBOTNIK_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define ROBOTNIK_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define ROBOTNIK_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define ROBOTNIK_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROBOTNIK_MSGS_PUBLIC
    #define ROBOTNIK_MSGS_LOCAL
  #endif
  #define ROBOTNIK_MSGS_PUBLIC_TYPE
#endif
#endif  // ROBOTNIK_MSGS__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 14:28:47
// Copyright 2019-2020 The MathWorks, Inc.
