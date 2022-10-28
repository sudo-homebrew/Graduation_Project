#ifndef ML_CLASSIFIERS__VISIBILITY_CONTROL_H_
#define ML_CLASSIFIERS__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ML_CLASSIFIERS_EXPORT __attribute__ ((dllexport))
    #define ML_CLASSIFIERS_IMPORT __attribute__ ((dllimport))
  #else
    #define ML_CLASSIFIERS_EXPORT __declspec(dllexport)
    #define ML_CLASSIFIERS_IMPORT __declspec(dllimport)
  #endif
  #ifdef ML_CLASSIFIERS_BUILDING_LIBRARY
    #define ML_CLASSIFIERS_PUBLIC ML_CLASSIFIERS_EXPORT
  #else
    #define ML_CLASSIFIERS_PUBLIC ML_CLASSIFIERS_IMPORT
  #endif
  #define ML_CLASSIFIERS_PUBLIC_TYPE ML_CLASSIFIERS_PUBLIC
  #define ML_CLASSIFIERS_LOCAL
#else
  #define ML_CLASSIFIERS_EXPORT __attribute__ ((visibility("default")))
  #define ML_CLASSIFIERS_IMPORT
  #if __GNUC__ >= 4
    #define ML_CLASSIFIERS_PUBLIC __attribute__ ((visibility("default")))
    #define ML_CLASSIFIERS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ML_CLASSIFIERS_PUBLIC
    #define ML_CLASSIFIERS_LOCAL
  #endif
  #define ML_CLASSIFIERS_PUBLIC_TYPE
#endif
#endif  // ML_CLASSIFIERS__VISIBILITY_CONTROL_H_
// Generated 06-Mar-2020 18:00:52
// Copyright 2019-2020 The MathWorks, Inc.
