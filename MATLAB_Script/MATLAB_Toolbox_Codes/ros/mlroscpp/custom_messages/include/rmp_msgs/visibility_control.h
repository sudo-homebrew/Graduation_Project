#ifndef RMP_MSGS__VISIBILITY_CONTROL_H_
#define RMP_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RMP_MSGS_EXPORT __attribute__ ((dllexport))
    #define RMP_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define RMP_MSGS_EXPORT __declspec(dllexport)
    #define RMP_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef RMP_MSGS_BUILDING_LIBRARY
    #define RMP_MSGS_PUBLIC RMP_MSGS_EXPORT
  #else
    #define RMP_MSGS_PUBLIC RMP_MSGS_IMPORT
  #endif
  #define RMP_MSGS_PUBLIC_TYPE RMP_MSGS_PUBLIC
  #define RMP_MSGS_LOCAL
#else
  #define RMP_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define RMP_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define RMP_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define RMP_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RMP_MSGS_PUBLIC
    #define RMP_MSGS_LOCAL
  #endif
  #define RMP_MSGS_PUBLIC_TYPE
#endif
#endif  // RMP_MSGS__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 14:27:19
// Copyright 2019-2020 The MathWorks, Inc.
