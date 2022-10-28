// MATLABActClientInterface.hpp
// Copyright 2021 The MathWorks, Inc.

#ifndef MATLABAINTERFACECOMMON_H
#define MATLABAINTERFACECOMMON_H

#ifndef FOUNDATION_MATLABDATA_API
#include "MDArray.hpp"
#include "StructArray.hpp"
#include "TypedArrayRef.hpp"
#include "Struct.hpp"
#include "ArrayFactory.hpp"
#include "StructRef.hpp"
#include "Reference.hpp"
#endif

#include "multi_library_class_loader.hpp"
using namespace class_loader;
#ifndef MultiLibLoader
#define MultiLibLoader MultiLibraryClassLoader*
#endif

#ifndef DLL_IMPORT_SYM
#ifdef _MSC_VER
#define DLL_IMPORT_SYM __declspec(dllimport)
#else
#define DLL_IMPORT_SYM __attribute__((visibility("default")))
#endif
#endif

#ifndef FOUNDATION_MATLABDATA_API
typedef matlab::data::Array MDArray_T;
typedef matlab::data::ArrayFactory MDFactory_T;
#else
#ifndef Rosbag
typedef foundation::matlabdata::Array MDArray_T;
typedef foundation::matlabdata::standalone::ClientArrayFactory MDFactory_T;
#else
typedef foundation::matlabdata::Array MDArray_T;
typedef foundation::matlabdata::matlab::ServerArrayFactory MDFactory_T;
#endif
#endif

#if defined(__APPLE__)
// Clang has problem with intpr_t
#define CONVERT_INTPTR_T_FOR_MATLAB_ARRAY(val) (static_cast<int64_t>(val))
#define CONVERT_SIZE_T_FOR_MATLAB_ARRAY(val) (static_cast<uint64_t>(val))
#else
#define CONVERT_INTPTR_T_FOR_MATLAB_ARRAY(val) (val)
#define CONVERT_SIZE_T_FOR_MATLAB_ARRAY(val) (val)
#endif // defined(__APPLE__)

typedef bool (*SendDataToMATLABFunc_T)(void* sd, const std::vector<MDArray_T>& outData);

#endif // MATLABAINTERFACECOMMON_H
