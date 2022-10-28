#ifndef KINGFISHER_MSGS__VISIBILITY_CONTROL_H_
#define KINGFISHER_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define KINGFISHER_MSGS_EXPORT __attribute__ ((dllexport))
    #define KINGFISHER_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define KINGFISHER_MSGS_EXPORT __declspec(dllexport)
    #define KINGFISHER_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef KINGFISHER_MSGS_BUILDING_LIBRARY
    #define KINGFISHER_MSGS_PUBLIC KINGFISHER_MSGS_EXPORT
  #else
    #define KINGFISHER_MSGS_PUBLIC KINGFISHER_MSGS_IMPORT
  #endif
  #define KINGFISHER_MSGS_PUBLIC_TYPE KINGFISHER_MSGS_PUBLIC
  #define KINGFISHER_MSGS_LOCAL
#else
  #define KINGFISHER_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define KINGFISHER_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define KINGFISHER_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define KINGFISHER_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define KINGFISHER_MSGS_PUBLIC
    #define KINGFISHER_MSGS_LOCAL
  #endif
  #define KINGFISHER_MSGS_PUBLIC_TYPE
#endif
#endif  // KINGFISHER_MSGS__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 14:01:06
// Copyright 2019-2020 The MathWorks, Inc.
