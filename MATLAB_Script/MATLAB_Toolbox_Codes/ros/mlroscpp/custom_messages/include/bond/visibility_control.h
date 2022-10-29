#ifndef BOND__VISIBILITY_CONTROL_H_
#define BOND__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define BOND_EXPORT __attribute__ ((dllexport))
    #define BOND_IMPORT __attribute__ ((dllimport))
  #else
    #define BOND_EXPORT __declspec(dllexport)
    #define BOND_IMPORT __declspec(dllimport)
  #endif
  #ifdef BOND_BUILDING_LIBRARY
    #define BOND_PUBLIC BOND_EXPORT
  #else
    #define BOND_PUBLIC BOND_IMPORT
  #endif
  #define BOND_PUBLIC_TYPE BOND_PUBLIC
  #define BOND_LOCAL
#else
  #define BOND_EXPORT __attribute__ ((visibility("default")))
  #define BOND_IMPORT
  #if __GNUC__ >= 4
    #define BOND_PUBLIC __attribute__ ((visibility("default")))
    #define BOND_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define BOND_PUBLIC
    #define BOND_LOCAL
  #endif
  #define BOND_PUBLIC_TYPE
#endif
#endif  // BOND__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 17:58:28
// Copyright 2019-2020 The MathWorks, Inc.
