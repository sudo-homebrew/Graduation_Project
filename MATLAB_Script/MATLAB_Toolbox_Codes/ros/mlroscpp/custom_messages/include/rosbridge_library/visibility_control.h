#ifndef ROSBRIDGE_LIBRARY__VISIBILITY_CONTROL_H_
#define ROSBRIDGE_LIBRARY__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSBRIDGE_LIBRARY_EXPORT __attribute__ ((dllexport))
    #define ROSBRIDGE_LIBRARY_IMPORT __attribute__ ((dllimport))
  #else
    #define ROSBRIDGE_LIBRARY_EXPORT __declspec(dllexport)
    #define ROSBRIDGE_LIBRARY_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROSBRIDGE_LIBRARY_BUILDING_LIBRARY
    #define ROSBRIDGE_LIBRARY_PUBLIC ROSBRIDGE_LIBRARY_EXPORT
  #else
    #define ROSBRIDGE_LIBRARY_PUBLIC ROSBRIDGE_LIBRARY_IMPORT
  #endif
  #define ROSBRIDGE_LIBRARY_PUBLIC_TYPE ROSBRIDGE_LIBRARY_PUBLIC
  #define ROSBRIDGE_LIBRARY_LOCAL
#else
  #define ROSBRIDGE_LIBRARY_EXPORT __attribute__ ((visibility("default")))
  #define ROSBRIDGE_LIBRARY_IMPORT
  #if __GNUC__ >= 4
    #define ROSBRIDGE_LIBRARY_PUBLIC __attribute__ ((visibility("default")))
    #define ROSBRIDGE_LIBRARY_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROSBRIDGE_LIBRARY_PUBLIC
    #define ROSBRIDGE_LIBRARY_LOCAL
  #endif
  #define ROSBRIDGE_LIBRARY_PUBLIC_TYPE
#endif
#endif  // ROSBRIDGE_LIBRARY__VISIBILITY_CONTROL_H_
// Generated 24-Apr-2020 15:42:28
// Copyright 2019-2020 The MathWorks, Inc.
