#ifndef CONTROL_TOOLBOX__VISIBILITY_CONTROL_H_
#define CONTROL_TOOLBOX__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CONTROL_TOOLBOX_EXPORT __attribute__ ((dllexport))
    #define CONTROL_TOOLBOX_IMPORT __attribute__ ((dllimport))
  #else
    #define CONTROL_TOOLBOX_EXPORT __declspec(dllexport)
    #define CONTROL_TOOLBOX_IMPORT __declspec(dllimport)
  #endif
  #ifdef CONTROL_TOOLBOX_BUILDING_LIBRARY
    #define CONTROL_TOOLBOX_PUBLIC CONTROL_TOOLBOX_EXPORT
  #else
    #define CONTROL_TOOLBOX_PUBLIC CONTROL_TOOLBOX_IMPORT
  #endif
  #define CONTROL_TOOLBOX_PUBLIC_TYPE CONTROL_TOOLBOX_PUBLIC
  #define CONTROL_TOOLBOX_LOCAL
#else
  #define CONTROL_TOOLBOX_EXPORT __attribute__ ((visibility("default")))
  #define CONTROL_TOOLBOX_IMPORT
  #if __GNUC__ >= 4
    #define CONTROL_TOOLBOX_PUBLIC __attribute__ ((visibility("default")))
    #define CONTROL_TOOLBOX_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CONTROL_TOOLBOX_PUBLIC
    #define CONTROL_TOOLBOX_LOCAL
  #endif
  #define CONTROL_TOOLBOX_PUBLIC_TYPE
#endif
#endif  // CONTROL_TOOLBOX__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 13:02:22
// Copyright 2019-2020 The MathWorks, Inc.
