// MATLABSubscriberInterface.hpp
// Copyright 2019-2021 The MathWorks, Inc.

#ifndef MATLABSVCSERVERINTEFACE_H
#define MATLABSVCSERVERINTEFACE_H

#ifndef FOUNDATION_MATLABDATA_API
#include "MDArray.hpp"
#include "StructArray.hpp"
#include "TypedArrayRef.hpp"
#include "Struct.hpp"
#include "ArrayFactory.hpp"
#include "StructRef.hpp"
#include "Reference.hpp"
#endif

#include "class_loader/multi_library_class_loader.hpp"
using namespace class_loader;
#define MultiLibLoader MultiLibraryClassLoader*

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
typedef foundation::matlabdata::Array MDArray_T;
typedef foundation::matlabdata::standalone::ClientArrayFactory MDFactory_T;
#endif

#if defined(__APPLE__)
// Clang has problem with intptr_t
#define CONVERT_INTPTR_T_FOR_MATLAB_ARRAY(val) (static_cast<int64_t>(val))
#define CONVERT_SIZE_T_FOR_MATLAB_ARRAY(val) (static_cast<uint64_t>(val))
#else
#define CONVERT_INTPTR_T_FOR_MATLAB_ARRAY(val) (val)
#define CONVERT_SIZE_T_FOR_MATLAB_ARRAY(val) (val)
#endif // defined(__APPLE__)

typedef bool (*SendDataToMATLABFunc_T)(void* sd, const std::vector<MDArray_T>& outData);
class MATLABROS2MsgInterfaceBase;
class MATLABSvcServerInterface {
  protected:
    MDFactory_T mFactory;
    bool mCallbackError;
    
  public:
    RCLCPP_SMART_PTR_DEFINITIONS(MATLABSvcServerInterface)
    MultiLibLoader mMultiLibLoader;
    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* mCommonObjMap;
    explicit MATLABSvcServerInterface() {
    }

    virtual ~MATLABSvcServerInterface() {
    }

    virtual intptr_t createSvcServer(const std::string& /* Svc_name */,
                                     rclcpp::Node::SharedPtr /* node */,
                                     const rclcpp::QoS& /* qos */,
                                     void* /* sd */,
                                     SendDataToMATLABFunc_T /* sendDataToMATLABFunc */,
                                     const intptr_t /* hSvcServer */) {
        return 0;
    }
    
    virtual void setCommonObjMap(std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* ){}
    
    virtual void appendAndSendToMATLAB(void* sd,
                                       SendDataToMATLABFunc_T sendDataToMATLABFunc,
                                       MDArray_T arr,
                                       const intptr_t hSvcServer,
                                       const std::shared_ptr<rmw_request_id_t> request_header) {
        (void)request_header;
        auto hndlArr = mFactory.createStructArray({1, 1}, {"handle"});
        hndlArr[0]["handle"] = mFactory.createScalar(CONVERT_INTPTR_T_FOR_MATLAB_ARRAY(hSvcServer));
        std::vector<MDArray_T> data{arr, hndlArr};
        sendDataToMATLABFunc(sd, data);
    }

#ifndef FOUNDATION_MATLABDATA_API
    virtual bool setResponseFromMatlab(const matlab::data::StructArray& /*arr*/)
#else
    virtual bool setResponseFromMatlab(const foundation::matlabdata::StructArray& /*arr*/)
#endif
    {
        return false; 
    }
    
    virtual void setCallbackError(bool callbackError) {
        mCallbackError = callbackError;
    }
     
    virtual bool getCallbackError() {
        return mCallbackError;
    }
};
#endif // MATLABSVCSERVERINTEFACE_H
