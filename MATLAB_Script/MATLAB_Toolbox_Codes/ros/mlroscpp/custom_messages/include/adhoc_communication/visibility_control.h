#ifndef ADHOC_COMMUNICATION__VISIBILITY_CONTROL_H_
#define ADHOC_COMMUNICATION__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ADHOC_COMMUNICATION_EXPORT __attribute__ ((dllexport))
    #define ADHOC_COMMUNICATION_IMPORT __attribute__ ((dllimport))
  #else
    #define ADHOC_COMMUNICATION_EXPORT __declspec(dllexport)
    #define ADHOC_COMMUNICATION_IMPORT __declspec(dllimport)
  #endif
  #ifdef ADHOC_COMMUNICATION_BUILDING_LIBRARY
    #define ADHOC_COMMUNICATION_PUBLIC ADHOC_COMMUNICATION_EXPORT
  #else
    #define ADHOC_COMMUNICATION_PUBLIC ADHOC_COMMUNICATION_IMPORT
  #endif
  #define ADHOC_COMMUNICATION_PUBLIC_TYPE ADHOC_COMMUNICATION_PUBLIC
  #define ADHOC_COMMUNICATION_LOCAL
#else
  #define ADHOC_COMMUNICATION_EXPORT __attribute__ ((visibility("default")))
  #define ADHOC_COMMUNICATION_IMPORT
  #if __GNUC__ >= 4
    #define ADHOC_COMMUNICATION_PUBLIC __attribute__ ((visibility("default")))
    #define ADHOC_COMMUNICATION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ADHOC_COMMUNICATION_PUBLIC
    #define ADHOC_COMMUNICATION_LOCAL
  #endif
  #define ADHOC_COMMUNICATION_PUBLIC_TYPE
#endif
#endif  // ADHOC_COMMUNICATION__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 12:34:31
// Copyright 2019-2020 The MathWorks, Inc.
