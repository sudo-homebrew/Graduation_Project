#ifndef BLOB__VISIBILITY_CONTROL_H_
#define BLOB__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define BLOB_EXPORT __attribute__ ((dllexport))
    #define BLOB_IMPORT __attribute__ ((dllimport))
  #else
    #define BLOB_EXPORT __declspec(dllexport)
    #define BLOB_IMPORT __declspec(dllimport)
  #endif
  #ifdef BLOB_BUILDING_LIBRARY
    #define BLOB_PUBLIC BLOB_EXPORT
  #else
    #define BLOB_PUBLIC BLOB_IMPORT
  #endif
  #define BLOB_PUBLIC_TYPE BLOB_PUBLIC
  #define BLOB_LOCAL
#else
  #define BLOB_EXPORT __attribute__ ((visibility("default")))
  #define BLOB_IMPORT
  #if __GNUC__ >= 4
    #define BLOB_PUBLIC __attribute__ ((visibility("default")))
    #define BLOB_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define BLOB_PUBLIC
    #define BLOB_LOCAL
  #endif
  #define BLOB_PUBLIC_TYPE
#endif
#endif  // BLOB__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 17:58:26
// Copyright 2019-2020 The MathWorks, Inc.
