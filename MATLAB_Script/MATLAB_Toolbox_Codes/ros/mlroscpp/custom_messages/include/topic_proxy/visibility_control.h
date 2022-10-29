#ifndef TOPIC_PROXY__VISIBILITY_CONTROL_H_
#define TOPIC_PROXY__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TOPIC_PROXY_EXPORT __attribute__ ((dllexport))
    #define TOPIC_PROXY_IMPORT __attribute__ ((dllimport))
  #else
    #define TOPIC_PROXY_EXPORT __declspec(dllexport)
    #define TOPIC_PROXY_IMPORT __declspec(dllimport)
  #endif
  #ifdef TOPIC_PROXY_BUILDING_LIBRARY
    #define TOPIC_PROXY_PUBLIC TOPIC_PROXY_EXPORT
  #else
    #define TOPIC_PROXY_PUBLIC TOPIC_PROXY_IMPORT
  #endif
  #define TOPIC_PROXY_PUBLIC_TYPE TOPIC_PROXY_PUBLIC
  #define TOPIC_PROXY_LOCAL
#else
  #define TOPIC_PROXY_EXPORT __attribute__ ((visibility("default")))
  #define TOPIC_PROXY_IMPORT
  #if __GNUC__ >= 4
    #define TOPIC_PROXY_PUBLIC __attribute__ ((visibility("default")))
    #define TOPIC_PROXY_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TOPIC_PROXY_PUBLIC
    #define TOPIC_PROXY_LOCAL
  #endif
  #define TOPIC_PROXY_PUBLIC_TYPE
#endif
#endif  // TOPIC_PROXY__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:03:07
// Copyright 2019-2020 The MathWorks, Inc.
