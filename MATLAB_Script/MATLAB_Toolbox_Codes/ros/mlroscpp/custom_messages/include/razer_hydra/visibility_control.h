#ifndef RAZER_HYDRA__VISIBILITY_CONTROL_H_
#define RAZER_HYDRA__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RAZER_HYDRA_EXPORT __attribute__ ((dllexport))
    #define RAZER_HYDRA_IMPORT __attribute__ ((dllimport))
  #else
    #define RAZER_HYDRA_EXPORT __declspec(dllexport)
    #define RAZER_HYDRA_IMPORT __declspec(dllimport)
  #endif
  #ifdef RAZER_HYDRA_BUILDING_LIBRARY
    #define RAZER_HYDRA_PUBLIC RAZER_HYDRA_EXPORT
  #else
    #define RAZER_HYDRA_PUBLIC RAZER_HYDRA_IMPORT
  #endif
  #define RAZER_HYDRA_PUBLIC_TYPE RAZER_HYDRA_PUBLIC
  #define RAZER_HYDRA_LOCAL
#else
  #define RAZER_HYDRA_EXPORT __attribute__ ((visibility("default")))
  #define RAZER_HYDRA_IMPORT
  #if __GNUC__ >= 4
    #define RAZER_HYDRA_PUBLIC __attribute__ ((visibility("default")))
    #define RAZER_HYDRA_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RAZER_HYDRA_PUBLIC
    #define RAZER_HYDRA_LOCAL
  #endif
  #define RAZER_HYDRA_PUBLIC_TYPE
#endif
#endif  // RAZER_HYDRA__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:01:54
// Copyright 2019-2020 The MathWorks, Inc.
