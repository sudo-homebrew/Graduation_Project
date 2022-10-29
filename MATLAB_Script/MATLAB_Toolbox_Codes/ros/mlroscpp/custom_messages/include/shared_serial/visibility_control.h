#ifndef SHARED_SERIAL__VISIBILITY_CONTROL_H_
#define SHARED_SERIAL__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SHARED_SERIAL_EXPORT __attribute__ ((dllexport))
    #define SHARED_SERIAL_IMPORT __attribute__ ((dllimport))
  #else
    #define SHARED_SERIAL_EXPORT __declspec(dllexport)
    #define SHARED_SERIAL_IMPORT __declspec(dllimport)
  #endif
  #ifdef SHARED_SERIAL_BUILDING_LIBRARY
    #define SHARED_SERIAL_PUBLIC SHARED_SERIAL_EXPORT
  #else
    #define SHARED_SERIAL_PUBLIC SHARED_SERIAL_IMPORT
  #endif
  #define SHARED_SERIAL_PUBLIC_TYPE SHARED_SERIAL_PUBLIC
  #define SHARED_SERIAL_LOCAL
#else
  #define SHARED_SERIAL_EXPORT __attribute__ ((visibility("default")))
  #define SHARED_SERIAL_IMPORT
  #if __GNUC__ >= 4
    #define SHARED_SERIAL_PUBLIC __attribute__ ((visibility("default")))
    #define SHARED_SERIAL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SHARED_SERIAL_PUBLIC
    #define SHARED_SERIAL_LOCAL
  #endif
  #define SHARED_SERIAL_PUBLIC_TYPE
#endif
#endif  // SHARED_SERIAL__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:02:39
// Copyright 2019-2020 The MathWorks, Inc.
