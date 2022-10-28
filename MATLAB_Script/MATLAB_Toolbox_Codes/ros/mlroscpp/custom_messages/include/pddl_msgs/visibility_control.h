#ifndef PDDL_MSGS__VISIBILITY_CONTROL_H_
#define PDDL_MSGS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PDDL_MSGS_EXPORT __attribute__ ((dllexport))
    #define PDDL_MSGS_IMPORT __attribute__ ((dllimport))
  #else
    #define PDDL_MSGS_EXPORT __declspec(dllexport)
    #define PDDL_MSGS_IMPORT __declspec(dllimport)
  #endif
  #ifdef PDDL_MSGS_BUILDING_LIBRARY
    #define PDDL_MSGS_PUBLIC PDDL_MSGS_EXPORT
  #else
    #define PDDL_MSGS_PUBLIC PDDL_MSGS_IMPORT
  #endif
  #define PDDL_MSGS_PUBLIC_TYPE PDDL_MSGS_PUBLIC
  #define PDDL_MSGS_LOCAL
#else
  #define PDDL_MSGS_EXPORT __attribute__ ((visibility("default")))
  #define PDDL_MSGS_IMPORT
  #if __GNUC__ >= 4
    #define PDDL_MSGS_PUBLIC __attribute__ ((visibility("default")))
    #define PDDL_MSGS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PDDL_MSGS_PUBLIC
    #define PDDL_MSGS_LOCAL
  #endif
  #define PDDL_MSGS_PUBLIC_TYPE
#endif
#endif  // PDDL_MSGS__VISIBILITY_CONTROL_H_
// Generated 27-Oct-2021 14:26:39
// Copyright 2019-2020 The MathWorks, Inc.
