#ifndef HUMANOID_NAV_MSGS__VISIBILITY_CONTROL_H_
#define HUMANOID_NAV_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define HUMANOID_NAV_MSGS_EXPORT __attribute__ ((dllexport))
    #define HUMANOID_NAV_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define HUMANOID_NAV_MSGS_EXPORT __declspec(dllexport)
    #define HUMANOID_NAV_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef HUMANOID_NAV_MSGS_BUILDING_LIBRARY
    #define HUMANOID_NAV_MSGS_PUBLIC HUMANOID_NAV_MSGS_EXPORT
  #else
    #define HUMANOID_NAV_MSGS_PUBLIC HUMANOID_NAV_MSGS_IMPORT
  #endif
  #define HUMANOID_NAV_MSGS_PUBLIC_TYPE HUMANOID_NAV_MSGS_PUBLIC
  #define HUMANOID_NAV_MSGS_LOCAL
#else
  #define HUMANOID_NAV_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define HUMANOID_NAV_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define HUMANOID_NAV_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define HUMANOID_NAV_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define HUMANOID_NAV_MSGS_PUBLIC
    #define HUMANOID_NAV_MSGS_LOCAL
  #endif
  #define HUMANOID_NAV_MSGS_PUBLIC_TYPE
#endif
#endif  // HUMANOID_NAV_MSGS__VISIBILITY_CONTROL_H_
// Generated 12-Apr-2020 11:46:32
// Copyright 2019-2020 The MathWorks, Inc.
