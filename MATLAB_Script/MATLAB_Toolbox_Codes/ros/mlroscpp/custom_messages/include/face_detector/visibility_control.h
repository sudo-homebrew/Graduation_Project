#ifndef FACE_DETECTOR__VISIBILITY_CONTROL_H_
#define FACE_DETECTOR__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define FACE_DETECTOR_EXPORT __attribute__ ((dllexport))
    #define FACE_DETECTOR_IMPORT __attribute__ ((dllimport))
  #else
    #define FACE_DETECTOR_EXPORT __declspec(dllexport)
    #define FACE_DETECTOR_IMPORT __declspec(dllimport)
  #endif
  #ifdef FACE_DETECTOR_BUILDING_LIBRARY
    #define FACE_DETECTOR_PUBLIC FACE_DETECTOR_EXPORT
  #else
    #define FACE_DETECTOR_PUBLIC FACE_DETECTOR_IMPORT
  #endif
  #define FACE_DETECTOR_PUBLIC_TYPE FACE_DETECTOR_PUBLIC
  #define FACE_DETECTOR_LOCAL
#else
  #define FACE_DETECTOR_EXPORT __attribute__ ((visibility("default")))
  #define FACE_DETECTOR_IMPORT
  #if __GNUC__ >= 4
    #define FACE_DETECTOR_PUBLIC __attribute__ ((visibility("default")))
    #define FACE_DETECTOR_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define FACE_DETECTOR_PUBLIC
    #define FACE_DETECTOR_LOCAL
  #endif
  #define FACE_DETECTOR_PUBLIC_TYPE
#endif
#endif  // FACE_DETECTOR__VISIBILITY_CONTROL_H_
// Generated 12-Apr-2020 11:46:30
// Copyright 2019-2020 The MathWorks, Inc.
