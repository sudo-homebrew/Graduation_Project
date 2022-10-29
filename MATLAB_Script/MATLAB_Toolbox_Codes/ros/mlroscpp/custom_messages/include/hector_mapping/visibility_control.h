#ifndef HECTOR_MAPPING__VISIBILITY_CONTROL_H_
#define HECTOR_MAPPING__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define HECTOR_MAPPING_EXPORT __attribute__ ((dllexport))
    #define HECTOR_MAPPING_IMPORT __attribute__ ((dllimport))
  #else
    #define HECTOR_MAPPING_EXPORT __declspec(dllexport)
    #define HECTOR_MAPPING_IMPORT __declspec(dllimport)
  #endif
  #ifdef HECTOR_MAPPING_BUILDING_LIBRARY
    #define HECTOR_MAPPING_PUBLIC HECTOR_MAPPING_EXPORT
  #else
    #define HECTOR_MAPPING_PUBLIC HECTOR_MAPPING_IMPORT
  #endif
  #define HECTOR_MAPPING_PUBLIC_TYPE HECTOR_MAPPING_PUBLIC
  #define HECTOR_MAPPING_LOCAL
#else
  #define HECTOR_MAPPING_EXPORT __attribute__ ((visibility("default")))
  #define HECTOR_MAPPING_IMPORT
  #if __GNUC__ >= 4
    #define HECTOR_MAPPING_PUBLIC __attribute__ ((visibility("default")))
    #define HECTOR_MAPPING_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define HECTOR_MAPPING_PUBLIC
    #define HECTOR_MAPPING_LOCAL
  #endif
  #define HECTOR_MAPPING_PUBLIC_TYPE
#endif
#endif  // HECTOR_MAPPING__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 13:50:40
// Copyright 2019-2020 The MathWorks, Inc.
