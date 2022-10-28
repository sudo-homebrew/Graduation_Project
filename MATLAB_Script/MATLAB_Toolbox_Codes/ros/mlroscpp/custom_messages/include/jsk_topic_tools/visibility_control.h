#ifndef JSK_TOPIC_TOOLS__VISIBILITY_CONTROL_H_
#define JSK_TOPIC_TOOLS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define JSK_TOPIC_TOOLS_EXPORT __attribute__ ((dllexport))
    #define JSK_TOPIC_TOOLS_IMPORT __attribute__ ((dllimport))
  #else
    #define JSK_TOPIC_TOOLS_EXPORT __declspec(dllexport)
    #define JSK_TOPIC_TOOLS_IMPORT __declspec(dllimport)
  #endif
  #ifdef JSK_TOPIC_TOOLS_BUILDING_LIBRARY
    #define JSK_TOPIC_TOOLS_PUBLIC JSK_TOPIC_TOOLS_EXPORT
  #else
    #define JSK_TOPIC_TOOLS_PUBLIC JSK_TOPIC_TOOLS_IMPORT
  #endif
  #define JSK_TOPIC_TOOLS_PUBLIC_TYPE JSK_TOPIC_TOOLS_PUBLIC
  #define JSK_TOPIC_TOOLS_LOCAL
#else
  #define JSK_TOPIC_TOOLS_EXPORT __attribute__ ((visibility("default")))
  #define JSK_TOPIC_TOOLS_IMPORT
  #if __GNUC__ >= 4
    #define JSK_TOPIC_TOOLS_PUBLIC __attribute__ ((visibility("default")))
    #define JSK_TOPIC_TOOLS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define JSK_TOPIC_TOOLS_PUBLIC
    #define JSK_TOPIC_TOOLS_LOCAL
  #endif
  #define JSK_TOPIC_TOOLS_PUBLIC_TYPE
#endif
#endif  // JSK_TOPIC_TOOLS__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 14:01:02
// Copyright 2019-2020 The MathWorks, Inc.
