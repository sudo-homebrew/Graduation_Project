#ifndef SPEECH_RECOGNITION_MSGS__VISIBILITY_CONTROL_H_
#define SPEECH_RECOGNITION_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SPEECH_RECOGNITION_MSGS_EXPORT __attribute__ ((dllexport))
    #define SPEECH_RECOGNITION_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define SPEECH_RECOGNITION_MSGS_EXPORT __declspec(dllexport)
    #define SPEECH_RECOGNITION_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef SPEECH_RECOGNITION_MSGS_BUILDING_LIBRARY
    #define SPEECH_RECOGNITION_MSGS_PUBLIC SPEECH_RECOGNITION_MSGS_EXPORT
  #else
    #define SPEECH_RECOGNITION_MSGS_PUBLIC SPEECH_RECOGNITION_MSGS_IMPORT
  #endif
  #define SPEECH_RECOGNITION_MSGS_PUBLIC_TYPE SPEECH_RECOGNITION_MSGS_PUBLIC
  #define SPEECH_RECOGNITION_MSGS_LOCAL
#else
  #define SPEECH_RECOGNITION_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define SPEECH_RECOGNITION_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define SPEECH_RECOGNITION_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define SPEECH_RECOGNITION_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SPEECH_RECOGNITION_MSGS_PUBLIC
    #define SPEECH_RECOGNITION_MSGS_LOCAL
  #endif
  #define SPEECH_RECOGNITION_MSGS_PUBLIC_TYPE
#endif
#endif  // SPEECH_RECOGNITION_MSGS__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 14:36:41
// Copyright 2019-2020 The MathWorks, Inc.
