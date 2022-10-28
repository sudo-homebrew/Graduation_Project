#ifndef THEORA_IMAGE_TRANSPORT__VISIBILITY_CONTROL_H_
#define THEORA_IMAGE_TRANSPORT__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define THEORA_IMAGE_TRANSPORT_EXPORT __attribute__ ((dllexport))
    #define THEORA_IMAGE_TRANSPORT_IMPORT __attribute__ ((dllimport))
  #else
    #define THEORA_IMAGE_TRANSPORT_EXPORT __declspec(dllexport)
    #define THEORA_IMAGE_TRANSPORT_IMPORT __declspec(dllimport)
  #endif
  #ifdef THEORA_IMAGE_TRANSPORT_BUILDING_LIBRARY
    #define THEORA_IMAGE_TRANSPORT_PUBLIC THEORA_IMAGE_TRANSPORT_EXPORT
  #else
    #define THEORA_IMAGE_TRANSPORT_PUBLIC THEORA_IMAGE_TRANSPORT_IMPORT
  #endif
  #define THEORA_IMAGE_TRANSPORT_PUBLIC_TYPE THEORA_IMAGE_TRANSPORT_PUBLIC
  #define THEORA_IMAGE_TRANSPORT_LOCAL
#else
  #define THEORA_IMAGE_TRANSPORT_EXPORT __attribute__ ((visibility("default")))
  #define THEORA_IMAGE_TRANSPORT_IMPORT
  #if __GNUC__ >= 4
    #define THEORA_IMAGE_TRANSPORT_PUBLIC __attribute__ ((visibility("default")))
    #define THEORA_IMAGE_TRANSPORT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define THEORA_IMAGE_TRANSPORT_PUBLIC
    #define THEORA_IMAGE_TRANSPORT_LOCAL
  #endif
  #define THEORA_IMAGE_TRANSPORT_PUBLIC_TYPE
#endif
#endif  // THEORA_IMAGE_TRANSPORT__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:03:05
// Copyright 2019-2020 The MathWorks, Inc.
