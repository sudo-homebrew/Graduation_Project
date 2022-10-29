#ifndef GRAPH_MSGS__VISIBILITY_CONTROL_H_
#define GRAPH_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GRAPH_MSGS_EXPORT __attribute__ ((dllexport))
    #define GRAPH_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define GRAPH_MSGS_EXPORT __declspec(dllexport)
    #define GRAPH_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef GRAPH_MSGS_BUILDING_LIBRARY
    #define GRAPH_MSGS_PUBLIC GRAPH_MSGS_EXPORT
  #else
    #define GRAPH_MSGS_PUBLIC GRAPH_MSGS_IMPORT
  #endif
  #define GRAPH_MSGS_PUBLIC_TYPE GRAPH_MSGS_PUBLIC
  #define GRAPH_MSGS_LOCAL
#else
  #define GRAPH_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define GRAPH_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define GRAPH_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define GRAPH_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GRAPH_MSGS_PUBLIC
    #define GRAPH_MSGS_LOCAL
  #endif
  #define GRAPH_MSGS_PUBLIC_TYPE
#endif
#endif  // GRAPH_MSGS__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 13:50:32
// Copyright 2019-2020 The MathWorks, Inc.
