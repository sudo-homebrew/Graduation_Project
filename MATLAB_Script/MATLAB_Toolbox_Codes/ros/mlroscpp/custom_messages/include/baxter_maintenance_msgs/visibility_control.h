#ifndef BAXTER_MAINTENANCE_MSGS__VISIBILITY_CONTROL_H_
#define BAXTER_MAINTENANCE_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define BAXTER_MAINTENANCE_MSGS_EXPORT __attribute__ ((dllexport))
    #define BAXTER_MAINTENANCE_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define BAXTER_MAINTENANCE_MSGS_EXPORT __declspec(dllexport)
    #define BAXTER_MAINTENANCE_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef BAXTER_MAINTENANCE_MSGS_BUILDING_LIBRARY
    #define BAXTER_MAINTENANCE_MSGS_PUBLIC BAXTER_MAINTENANCE_MSGS_EXPORT
  #else
    #define BAXTER_MAINTENANCE_MSGS_PUBLIC BAXTER_MAINTENANCE_MSGS_IMPORT
  #endif
  #define BAXTER_MAINTENANCE_MSGS_PUBLIC_TYPE BAXTER_MAINTENANCE_MSGS_PUBLIC
  #define BAXTER_MAINTENANCE_MSGS_LOCAL
#else
  #define BAXTER_MAINTENANCE_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define BAXTER_MAINTENANCE_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define BAXTER_MAINTENANCE_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define BAXTER_MAINTENANCE_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define BAXTER_MAINTENANCE_MSGS_PUBLIC
    #define BAXTER_MAINTENANCE_MSGS_LOCAL
  #endif
  #define BAXTER_MAINTENANCE_MSGS_PUBLIC_TYPE
#endif
#endif  // BAXTER_MAINTENANCE_MSGS__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 12:35:19
// Copyright 2019-2020 The MathWorks, Inc.
