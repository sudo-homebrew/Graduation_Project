#ifndef KEYBOARD__VISIBILITY_CONTROL_H_
#define KEYBOARD__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define KEYBOARD_EXPORT __attribute__ ((dllexport))
    #define KEYBOARD_IMPORT __attribute__ ((dllimport))
  #else
    #define KEYBOARD_EXPORT __declspec(dllexport)
    #define KEYBOARD_IMPORT __declspec(dllimport)
  #endif
  #ifdef KEYBOARD_BUILDING_LIBRARY
    #define KEYBOARD_PUBLIC KEYBOARD_EXPORT
  #else
    #define KEYBOARD_PUBLIC KEYBOARD_IMPORT
  #endif
  #define KEYBOARD_PUBLIC_TYPE KEYBOARD_PUBLIC
  #define KEYBOARD_LOCAL
#else
  #define KEYBOARD_EXPORT __attribute__ ((visibility("default")))
  #define KEYBOARD_IMPORT
  #if __GNUC__ >= 4
    #define KEYBOARD_PUBLIC __attribute__ ((visibility("default")))
    #define KEYBOARD_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define KEYBOARD_PUBLIC
    #define KEYBOARD_LOCAL
  #endif
  #define KEYBOARD_PUBLIC_TYPE
#endif
#endif  // KEYBOARD__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:00:30
// Copyright 2019-2020 The MathWorks, Inc.
