#ifndef LASER_ASSEMBLER__VISIBILITY_CONTROL_H_
#define LASER_ASSEMBLER__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define LASER_ASSEMBLER_EXPORT __attribute__ ((dllexport))
    #define LASER_ASSEMBLER_IMPORT __attribute__ ((dllimport))
  #else
    #define LASER_ASSEMBLER_EXPORT __declspec(dllexport)
    #define LASER_ASSEMBLER_IMPORT __declspec(dllimport)
  #endif
  #ifdef LASER_ASSEMBLER_BUILDING_LIBRARY
    #define LASER_ASSEMBLER_PUBLIC LASER_ASSEMBLER_EXPORT
  #else
    #define LASER_ASSEMBLER_PUBLIC LASER_ASSEMBLER_IMPORT
  #endif
  #define LASER_ASSEMBLER_PUBLIC_TYPE LASER_ASSEMBLER_PUBLIC
  #define LASER_ASSEMBLER_LOCAL
#else
  #define LASER_ASSEMBLER_EXPORT __attribute__ ((visibility("default")))
  #define LASER_ASSEMBLER_IMPORT
  #if __GNUC__ >= 4
    #define LASER_ASSEMBLER_PUBLIC __attribute__ ((visibility("default")))
    #define LASER_ASSEMBLER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define LASER_ASSEMBLER_PUBLIC
    #define LASER_ASSEMBLER_LOCAL
  #endif
  #define LASER_ASSEMBLER_PUBLIC_TYPE
#endif
#endif  // LASER_ASSEMBLER__VISIBILITY_CONTROL_H_
// Generated 25-Mar-2020 20:24:05
// Copyright 2019-2020 The MathWorks, Inc.
