#ifndef ROVIO_SHARED__VISIBILITY_CONTROL_H_
#define ROVIO_SHARED__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROVIO_SHARED_EXPORT __attribute__ ((dllexport))
    #define ROVIO_SHARED_IMPORT __attribute__ ((dllimport))
  #else
    #define ROVIO_SHARED_EXPORT __declspec(dllexport)
    #define ROVIO_SHARED_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROVIO_SHARED_BUILDING_LIBRARY
    #define ROVIO_SHARED_PUBLIC ROVIO_SHARED_EXPORT
  #else
    #define ROVIO_SHARED_PUBLIC ROVIO_SHARED_IMPORT
  #endif
  #define ROVIO_SHARED_PUBLIC_TYPE ROVIO_SHARED_PUBLIC
  #define ROVIO_SHARED_LOCAL
#else
  #define ROVIO_SHARED_EXPORT __attribute__ ((visibility("default")))
  #define ROVIO_SHARED_IMPORT
  #if __GNUC__ >= 4
    #define ROVIO_SHARED_PUBLIC __attribute__ ((visibility("default")))
    #define ROVIO_SHARED_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROVIO_SHARED_PUBLIC
    #define ROVIO_SHARED_LOCAL
  #endif
  #define ROVIO_SHARED_PUBLIC_TYPE
#endif
#endif  // ROVIO_SHARED__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:02:22
// Copyright 2019-2020 The MathWorks, Inc.
