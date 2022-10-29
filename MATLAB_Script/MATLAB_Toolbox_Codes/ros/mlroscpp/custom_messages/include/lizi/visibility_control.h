#ifndef LIZI__VISIBILITY_CONTROL_H_
#define LIZI__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define LIZI_EXPORT __attribute__ ((dllexport))
    #define LIZI_IMPORT __attribute__ ((dllimport))
  #else
    #define LIZI_EXPORT __declspec(dllexport)
    #define LIZI_IMPORT __declspec(dllimport)
  #endif
  #ifdef LIZI_BUILDING_LIBRARY
    #define LIZI_PUBLIC LIZI_EXPORT
  #else
    #define LIZI_PUBLIC LIZI_IMPORT
  #endif
  #define LIZI_PUBLIC_TYPE LIZI_PUBLIC
  #define LIZI_LOCAL
#else
  #define LIZI_EXPORT __attribute__ ((visibility("default")))
  #define LIZI_IMPORT
  #if __GNUC__ >= 4
    #define LIZI_PUBLIC __attribute__ ((visibility("default")))
    #define LIZI_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define LIZI_PUBLIC
    #define LIZI_LOCAL
  #endif
  #define LIZI_PUBLIC_TYPE
#endif
#endif  // LIZI__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:00:43
// Copyright 2019-2020 The MathWorks, Inc.
