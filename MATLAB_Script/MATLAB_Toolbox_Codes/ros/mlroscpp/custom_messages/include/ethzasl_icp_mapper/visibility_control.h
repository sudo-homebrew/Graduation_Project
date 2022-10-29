#ifndef ETHZASL_ICP_MAPPER__VISIBILITY_CONTROL_H_
#define ETHZASL_ICP_MAPPER__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ETHZASL_ICP_MAPPER_EXPORT __attribute__ ((dllexport))
    #define ETHZASL_ICP_MAPPER_IMPORT __attribute__ ((dllimport))
  #else
    #define ETHZASL_ICP_MAPPER_EXPORT __declspec(dllexport)
    #define ETHZASL_ICP_MAPPER_IMPORT __declspec(dllimport)
  #endif
  #ifdef ETHZASL_ICP_MAPPER_BUILDING_LIBRARY
    #define ETHZASL_ICP_MAPPER_PUBLIC ETHZASL_ICP_MAPPER_EXPORT
  #else
    #define ETHZASL_ICP_MAPPER_PUBLIC ETHZASL_ICP_MAPPER_IMPORT
  #endif
  #define ETHZASL_ICP_MAPPER_PUBLIC_TYPE ETHZASL_ICP_MAPPER_PUBLIC
  #define ETHZASL_ICP_MAPPER_LOCAL
#else
  #define ETHZASL_ICP_MAPPER_EXPORT __attribute__ ((visibility("default")))
  #define ETHZASL_ICP_MAPPER_IMPORT
  #if __GNUC__ >= 4
    #define ETHZASL_ICP_MAPPER_PUBLIC __attribute__ ((visibility("default")))
    #define ETHZASL_ICP_MAPPER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ETHZASL_ICP_MAPPER_PUBLIC
    #define ETHZASL_ICP_MAPPER_LOCAL
  #endif
  #define ETHZASL_ICP_MAPPER_PUBLIC_TYPE
#endif
#endif  // ETHZASL_ICP_MAPPER__VISIBILITY_CONTROL_H_
// Generated 25-Mar-2020 20:24:03
// Copyright 2019-2020 The MathWorks, Inc.
