#ifndef MONGODB_STORE_MSGS__VISIBILITY_CONTROL_H_
#define MONGODB_STORE_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MONGODB_STORE_MSGS_EXPORT __attribute__ ((dllexport))
    #define MONGODB_STORE_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define MONGODB_STORE_MSGS_EXPORT __declspec(dllexport)
    #define MONGODB_STORE_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef MONGODB_STORE_MSGS_BUILDING_LIBRARY
    #define MONGODB_STORE_MSGS_PUBLIC MONGODB_STORE_MSGS_EXPORT
  #else
    #define MONGODB_STORE_MSGS_PUBLIC MONGODB_STORE_MSGS_IMPORT
  #endif
  #define MONGODB_STORE_MSGS_PUBLIC_TYPE MONGODB_STORE_MSGS_PUBLIC
  #define MONGODB_STORE_MSGS_LOCAL
#else
  #define MONGODB_STORE_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define MONGODB_STORE_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define MONGODB_STORE_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define MONGODB_STORE_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MONGODB_STORE_MSGS_PUBLIC
    #define MONGODB_STORE_MSGS_LOCAL
  #endif
  #define MONGODB_STORE_MSGS_PUBLIC_TYPE
#endif
#endif  // MONGODB_STORE_MSGS__VISIBILITY_CONTROL_H_
// Generated 12-Apr-2020 11:46:38
// Copyright 2019-2020 The MathWorks, Inc.
