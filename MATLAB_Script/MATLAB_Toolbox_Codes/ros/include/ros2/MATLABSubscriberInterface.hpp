// MATLABSubscriberInterface.hpp
// Copyright 2017-2021 The MathWorks, Inc.

#ifndef MATLABSUBSCRIBERINTEFACE_H
#define MATLABSUBSCRIBERINTEFACE_H

#ifndef FOUNDATION_MATLABDATA_API
#include "MDArray.hpp"
#include "StructArray.hpp"
#include "TypedArrayRef.hpp"
#include "Struct.hpp"
#include "ArrayFactory.hpp"
#include "StructRef.hpp"
#include "Reference.hpp"
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
typedef foundation::matlabdata::Array MDArray_T;
typedef foundation::matlabdata::standalone::ClientArrayFactory MDFactory_T;
#endif
#include "class_loader/multi_library_class_loader.hpp"
using namespace class_loader;
#define MultiLibLoader MultiLibraryClassLoader*

typedef bool (*SendDataToMATLABFunc_T)(void* sd, const std::vector<MDArray_T>& outData);
class MATLABROS2MsgInterfaceBase;
class MATLABSubscriberInterface {
  protected:
    MDFactory_T mFactory;

  public:
    RCLCPP_SMART_PTR_DEFINITIONS(MATLABSubscriberInterface)
    MultiLibLoader mMultiLibLoader;
    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* mCommonObjMap;
    explicit MATLABSubscriberInterface() {
    }

    virtual ~MATLABSubscriberInterface() {
    }

    virtual intptr_t createSubscription(const std::string& /* topic_name */,
                                        const rclcpp::QoS& /* qos_profile */,
                                        rclcpp::Node::SharedPtr /* node */,
                                        void* /* sd */,
                                        SendDataToMATLABFunc_T /* sendDataToMATLABFunc */,
                                        const bool /* incPubGid */) {
        return 0;
    }
    
    virtual void setCommonObjMap(std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* ){}
    
    virtual void appendAndSendToMATLAB(void* sd,
                                       SendDataToMATLABFunc_T sendDataToMATLABFunc,
                                       MDArray_T arr,
                                       const bool incPubGid,
                                       const rclcpp::MessageInfo & msgInfo) {
        if (incPubGid) {
            auto gid = mFactory.createArray<uint8_t>({1, RMW_GID_STORAGE_SIZE});
            std::copy(msgInfo.get_rmw_message_info().publisher_gid.data, msgInfo.get_rmw_message_info().publisher_gid.data + RMW_GID_STORAGE_SIZE,
                      gid.begin());
            auto gidArr = mFactory.createStructArray({1, 1}, {"publisher_gid"});
            gidArr[0]["publisher_gid"] = gid;

            std::vector<MDArray_T> data{arr, gidArr};
            sendDataToMATLABFunc(sd, data);
        } else {
            std::vector<MDArray_T> data{arr};
            sendDataToMATLABFunc(sd, data);
        }
    }
};

#endif // MATLABSUBSCRIBERINTEFACE_H
