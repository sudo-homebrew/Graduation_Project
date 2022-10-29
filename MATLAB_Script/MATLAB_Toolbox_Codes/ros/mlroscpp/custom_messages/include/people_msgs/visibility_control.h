#ifndef PEOPLE_MSGS__VISIBILITY_CONTROL_H_
#define PEOPLE_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PEOPLE_MSGS_EXPORT __attribute__ ((dllexport))
    #define PEOPLE_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define PEOPLE_MSGS_EXPORT __declspec(dllexport)
    #define PEOPLE_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef PEOPLE_MSGS_BUILDING_LIBRARY
    #define PEOPLE_MSGS_PUBLIC PEOPLE_MSGS_EXPORT
  #else
    #define PEOPLE_MSGS_PUBLIC PEOPLE_MSGS_IMPORT
  #endif
  #define PEOPLE_MSGS_PUBLIC_TYPE PEOPLE_MSGS_PUBLIC
  #define PEOPLE_MSGS_LOCAL
#else
  #define PEOPLE_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define PEOPLE_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define PEOPLE_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define PEOPLE_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PEOPLE_MSGS_PUBLIC
    #define PEOPLE_MSGS_LOCAL
  #endif
  #define PEOPLE_MSGS_PUBLIC_TYPE
#endif
#endif  // PEOPLE_MSGS__VISIBILITY_CONTROL_H_
// Generated 12-Apr-2020 11:46:42
// Copyright 2019-2020 The MathWorks, Inc.
