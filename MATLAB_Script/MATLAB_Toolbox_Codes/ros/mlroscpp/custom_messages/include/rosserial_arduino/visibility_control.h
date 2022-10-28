#ifndef ROSSERIAL_ARDUINO__VISIBILITY_CONTROL_H_
#define ROSSERIAL_ARDUINO__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSSERIAL_ARDUINO_EXPORT __attribute__ ((dllexport))
    #define ROSSERIAL_ARDUINO_IMPORT __attribute__ ((dllimport))
  #else
    #define ROSSERIAL_ARDUINO_EXPORT __declspec(dllexport)
    #define ROSSERIAL_ARDUINO_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROSSERIAL_ARDUINO_BUILDING_LIBRARY
    #define ROSSERIAL_ARDUINO_PUBLIC ROSSERIAL_ARDUINO_EXPORT
  #else
    #define ROSSERIAL_ARDUINO_PUBLIC ROSSERIAL_ARDUINO_IMPORT
  #endif
  #define ROSSERIAL_ARDUINO_PUBLIC_TYPE ROSSERIAL_ARDUINO_PUBLIC
  #define ROSSERIAL_ARDUINO_LOCAL
#else
  #define ROSSERIAL_ARDUINO_EXPORT __attribute__ ((visibility("default")))
  #define ROSSERIAL_ARDUINO_IMPORT
  #if __GNUC__ >= 4
    #define ROSSERIAL_ARDUINO_PUBLIC __attribute__ ((visibility("default")))
    #define ROSSERIAL_ARDUINO_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROSSERIAL_ARDUINO_PUBLIC
    #define ROSSERIAL_ARDUINO_LOCAL
  #endif
  #define ROSSERIAL_ARDUINO_PUBLIC_TYPE
#endif
#endif  // ROSSERIAL_ARDUINO__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 14:36:08
// Copyright 2019-2020 The MathWorks, Inc.
