#ifndef MICROSTRAIN_3DMGX2_IMU__VISIBILITY_CONTROL_H_
#define MICROSTRAIN_3DMGX2_IMU__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MICROSTRAIN_3DMGX2_IMU_EXPORT __attribute__ ((dllexport))
    #define MICROSTRAIN_3DMGX2_IMU_IMPORT __attribute__ ((dllimport))
  #else
    #define MICROSTRAIN_3DMGX2_IMU_EXPORT __declspec(dllexport)
    #define MICROSTRAIN_3DMGX2_IMU_IMPORT __declspec(dllimport)
  #endif
  #ifdef MICROSTRAIN_3DMGX2_IMU_BUILDING_LIBRARY
    #define MICROSTRAIN_3DMGX2_IMU_PUBLIC MICROSTRAIN_3DMGX2_IMU_EXPORT
  #else
    #define MICROSTRAIN_3DMGX2_IMU_PUBLIC MICROSTRAIN_3DMGX2_IMU_IMPORT
  #endif
  #define MICROSTRAIN_3DMGX2_IMU_PUBLIC_TYPE MICROSTRAIN_3DMGX2_IMU_PUBLIC
  #define MICROSTRAIN_3DMGX2_IMU_LOCAL
#else
  #define MICROSTRAIN_3DMGX2_IMU_EXPORT __attribute__ ((visibility("default")))
  #define MICROSTRAIN_3DMGX2_IMU_IMPORT
  #if __GNUC__ >= 4
    #define MICROSTRAIN_3DMGX2_IMU_PUBLIC __attribute__ ((visibility("default")))
    #define MICROSTRAIN_3DMGX2_IMU_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MICROSTRAIN_3DMGX2_IMU_PUBLIC
    #define MICROSTRAIN_3DMGX2_IMU_LOCAL
  #endif
  #define MICROSTRAIN_3DMGX2_IMU_PUBLIC_TYPE
#endif
#endif  // MICROSTRAIN_3DMGX2_IMU__VISIBILITY_CONTROL_H_
// Generated 25-Mar-2020 21:51:30
// Copyright 2019-2020 The MathWorks, Inc.
