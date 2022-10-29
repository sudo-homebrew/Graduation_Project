#ifndef COB_RELAYBOARD__VISIBILITY_CONTROL_H_
#define COB_RELAYBOARD__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define COB_RELAYBOARD_EXPORT __attribute__ ((dllexport))
    #define COB_RELAYBOARD_IMPORT __attribute__ ((dllimport))
  #else
    #define COB_RELAYBOARD_EXPORT __declspec(dllexport)
    #define COB_RELAYBOARD_IMPORT __declspec(dllimport)
  #endif
  #ifdef COB_RELAYBOARD_BUILDING_LIBRARY
    #define COB_RELAYBOARD_PUBLIC COB_RELAYBOARD_EXPORT
  #else
    #define COB_RELAYBOARD_PUBLIC COB_RELAYBOARD_IMPORT
  #endif
  #define COB_RELAYBOARD_PUBLIC_TYPE COB_RELAYBOARD_PUBLIC
  #define COB_RELAYBOARD_LOCAL
#else
  #define COB_RELAYBOARD_EXPORT __attribute__ ((visibility("default")))
  #define COB_RELAYBOARD_IMPORT
  #if __GNUC__ >= 4
    #define COB_RELAYBOARD_PUBLIC __attribute__ ((visibility("default")))
    #define COB_RELAYBOARD_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define COB_RELAYBOARD_PUBLIC
    #define COB_RELAYBOARD_LOCAL
  #endif
  #define COB_RELAYBOARD_PUBLIC_TYPE
#endif
#endif  // COB_RELAYBOARD__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 17:58:51
// Copyright 2019-2020 The MathWorks, Inc.
