#ifndef ARDRONE_AUTONOMY__VISIBILITY_CONTROL_H_
#define ARDRONE_AUTONOMY__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ARDRONE_AUTONOMY_EXPORT __attribute__ ((dllexport))
    #define ARDRONE_AUTONOMY_IMPORT __attribute__ ((dllimport))
  #else
    #define ARDRONE_AUTONOMY_EXPORT __declspec(dllexport)
    #define ARDRONE_AUTONOMY_IMPORT __declspec(dllimport)
  #endif
  #ifdef ARDRONE_AUTONOMY_BUILDING_LIBRARY
    #define ARDRONE_AUTONOMY_PUBLIC ARDRONE_AUTONOMY_EXPORT
  #else
    #define ARDRONE_AUTONOMY_PUBLIC ARDRONE_AUTONOMY_IMPORT
  #endif
  #define ARDRONE_AUTONOMY_PUBLIC_TYPE ARDRONE_AUTONOMY_PUBLIC
  #define ARDRONE_AUTONOMY_LOCAL
#else
  #define ARDRONE_AUTONOMY_EXPORT __attribute__ ((visibility("default")))
  #define ARDRONE_AUTONOMY_IMPORT
  #if __GNUC__ >= 4
    #define ARDRONE_AUTONOMY_PUBLIC __attribute__ ((visibility("default")))
    #define ARDRONE_AUTONOMY_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ARDRONE_AUTONOMY_PUBLIC
    #define ARDRONE_AUTONOMY_LOCAL
  #endif
  #define ARDRONE_AUTONOMY_PUBLIC_TYPE
#endif
#endif  // ARDRONE_AUTONOMY__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 17:58:13
// Copyright 2019-2020 The MathWorks, Inc.
