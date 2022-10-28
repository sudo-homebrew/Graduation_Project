#ifndef S3000_LASER__VISIBILITY_CONTROL_H_
#define S3000_LASER__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define S3000_LASER_EXPORT __attribute__ ((dllexport))
    #define S3000_LASER_IMPORT __attribute__ ((dllimport))
  #else
    #define S3000_LASER_EXPORT __declspec(dllexport)
    #define S3000_LASER_IMPORT __declspec(dllimport)
  #endif
  #ifdef S3000_LASER_BUILDING_LIBRARY
    #define S3000_LASER_PUBLIC S3000_LASER_EXPORT
  #else
    #define S3000_LASER_PUBLIC S3000_LASER_IMPORT
  #endif
  #define S3000_LASER_PUBLIC_TYPE S3000_LASER_PUBLIC
  #define S3000_LASER_LOCAL
#else
  #define S3000_LASER_EXPORT __attribute__ ((visibility("default")))
  #define S3000_LASER_IMPORT
  #if __GNUC__ >= 4
    #define S3000_LASER_PUBLIC __attribute__ ((visibility("default")))
    #define S3000_LASER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define S3000_LASER_PUBLIC
    #define S3000_LASER_LOCAL
  #endif
  #define S3000_LASER_PUBLIC_TYPE
#endif
#endif  // S3000_LASER__VISIBILITY_CONTROL_H_
// Generated 25-Mar-2020 22:24:58
// Copyright 2019-2020 The MathWorks, Inc.
