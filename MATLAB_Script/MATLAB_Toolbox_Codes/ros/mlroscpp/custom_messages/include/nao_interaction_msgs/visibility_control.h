#ifndef NAO_INTERACTION_MSGS__VISIBILITY_CONTROL_H_
#define NAO_INTERACTION_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define NAO_INTERACTION_MSGS_EXPORT __attribute__ ((dllexport))
    #define NAO_INTERACTION_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define NAO_INTERACTION_MSGS_EXPORT __declspec(dllexport)
    #define NAO_INTERACTION_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef NAO_INTERACTION_MSGS_BUILDING_LIBRARY
    #define NAO_INTERACTION_MSGS_PUBLIC NAO_INTERACTION_MSGS_EXPORT
  #else
    #define NAO_INTERACTION_MSGS_PUBLIC NAO_INTERACTION_MSGS_IMPORT
  #endif
  #define NAO_INTERACTION_MSGS_PUBLIC_TYPE NAO_INTERACTION_MSGS_PUBLIC
  #define NAO_INTERACTION_MSGS_LOCAL
#else
  #define NAO_INTERACTION_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define NAO_INTERACTION_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define NAO_INTERACTION_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define NAO_INTERACTION_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define NAO_INTERACTION_MSGS_PUBLIC
    #define NAO_INTERACTION_MSGS_LOCAL
  #endif
  #define NAO_INTERACTION_MSGS_PUBLIC_TYPE
#endif
#endif  // NAO_INTERACTION_MSGS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:01:02
// Copyright 2019-2020 The MathWorks, Inc.