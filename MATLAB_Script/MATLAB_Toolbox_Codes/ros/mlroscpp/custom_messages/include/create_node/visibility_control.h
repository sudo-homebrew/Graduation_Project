#ifndef CREATE_NODE__VISIBILITY_CONTROL_H_
#define CREATE_NODE__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CREATE_NODE_EXPORT __attribute__ ((dllexport))
    #define CREATE_NODE_IMPORT __attribute__ ((dllimport))
  #else
    #define CREATE_NODE_EXPORT __declspec(dllexport)
    #define CREATE_NODE_IMPORT __declspec(dllimport)
  #endif
  #ifdef CREATE_NODE_BUILDING_LIBRARY
    #define CREATE_NODE_PUBLIC CREATE_NODE_EXPORT
  #else
    #define CREATE_NODE_PUBLIC CREATE_NODE_IMPORT
  #endif
  #define CREATE_NODE_PUBLIC_TYPE CREATE_NODE_PUBLIC
  #define CREATE_NODE_LOCAL
#else
  #define CREATE_NODE_EXPORT __attribute__ ((visibility("default")))
  #define CREATE_NODE_IMPORT
  #if __GNUC__ >= 4
    #define CREATE_NODE_PUBLIC __attribute__ ((visibility("default")))
    #define CREATE_NODE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CREATE_NODE_PUBLIC
    #define CREATE_NODE_LOCAL
  #endif
  #define CREATE_NODE_PUBLIC_TYPE
#endif
#endif  // CREATE_NODE__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 17:59:03
// Copyright 2019-2020 The MathWorks, Inc.
