#ifndef PROGRAM_QUEUE__VISIBILITY_CONTROL_H_
#define PROGRAM_QUEUE__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PROGRAM_QUEUE_EXPORT __attribute__ ((dllexport))
    #define PROGRAM_QUEUE_IMPORT __attribute__ ((dllimport))
  #else
    #define PROGRAM_QUEUE_EXPORT __declspec(dllexport)
    #define PROGRAM_QUEUE_IMPORT __declspec(dllimport)
  #endif
  #ifdef PROGRAM_QUEUE_BUILDING_LIBRARY
    #define PROGRAM_QUEUE_PUBLIC PROGRAM_QUEUE_EXPORT
  #else
    #define PROGRAM_QUEUE_PUBLIC PROGRAM_QUEUE_IMPORT
  #endif
  #define PROGRAM_QUEUE_PUBLIC_TYPE PROGRAM_QUEUE_PUBLIC
  #define PROGRAM_QUEUE_LOCAL
#else
  #define PROGRAM_QUEUE_EXPORT __attribute__ ((visibility("default")))
  #define PROGRAM_QUEUE_IMPORT
  #if __GNUC__ >= 4
    #define PROGRAM_QUEUE_PUBLIC __attribute__ ((visibility("default")))
    #define PROGRAM_QUEUE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PROGRAM_QUEUE_PUBLIC
    #define PROGRAM_QUEUE_LOCAL
  #endif
  #define PROGRAM_QUEUE_PUBLIC_TYPE
#endif
#endif  // PROGRAM_QUEUE__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:01:49
// Copyright 2019-2020 The MathWorks, Inc.
