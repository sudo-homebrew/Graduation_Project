#ifndef MLN_ROBOSHERLOCK_MSGS__VISIBILITY_CONTROL_H_
#define MLN_ROBOSHERLOCK_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MLN_ROBOSHERLOCK_MSGS_EXPORT __attribute__ ((dllexport))
    #define MLN_ROBOSHERLOCK_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define MLN_ROBOSHERLOCK_MSGS_EXPORT __declspec(dllexport)
    #define MLN_ROBOSHERLOCK_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef MLN_ROBOSHERLOCK_MSGS_BUILDING_LIBRARY
    #define MLN_ROBOSHERLOCK_MSGS_PUBLIC MLN_ROBOSHERLOCK_MSGS_EXPORT
  #else
    #define MLN_ROBOSHERLOCK_MSGS_PUBLIC MLN_ROBOSHERLOCK_MSGS_IMPORT
  #endif
  #define MLN_ROBOSHERLOCK_MSGS_PUBLIC_TYPE MLN_ROBOSHERLOCK_MSGS_PUBLIC
  #define MLN_ROBOSHERLOCK_MSGS_LOCAL
#else
  #define MLN_ROBOSHERLOCK_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define MLN_ROBOSHERLOCK_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define MLN_ROBOSHERLOCK_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define MLN_ROBOSHERLOCK_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MLN_ROBOSHERLOCK_MSGS_PUBLIC
    #define MLN_ROBOSHERLOCK_MSGS_LOCAL
  #endif
  #define MLN_ROBOSHERLOCK_MSGS_PUBLIC_TYPE
#endif
#endif  // MLN_ROBOSHERLOCK_MSGS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:00:54
// Copyright 2019-2020 The MathWorks, Inc.
