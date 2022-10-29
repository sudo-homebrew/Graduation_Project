// MATLABROSMsgInterface.hpp
// Copyright 2020-2021 The MathWorks, Inc.

#ifndef MATLABROS2MSGINTERFACE1_H
#define MATLABROS2MSGINTERFACE1_H

#ifndef FOUNDATION_MATLABDATA_API
#include "MDArray.hpp"
#include "StructArray.hpp"
#include "TypedArrayRef.hpp"
#include "Struct.hpp"
#include "ArrayFactory.hpp"
#include "StructRef.hpp"
#include "Reference.hpp"
#endif
#ifndef FOUNDATION_MATLABDATA_API
typedef matlab::data::Array MDArray_T;
typedef matlab::data::ArrayFactory MDFactory_T;
#else
typedef foundation::matlabdata::Array MDArray_T;
typedef foundation::matlabdata::standalone::ClientArrayFactory MDFactory_T;
#endif

#ifndef DLL_IMPORT_SYM
#ifdef _MSC_VER
#define DLL_IMPORT_SYM __declspec(dllimport)
#else
#define DLL_IMPORT_SYM __attribute__((visibility("default")))
#endif
#endif

#include "class_loader/multi_library_class_loader.hpp"
using namespace class_loader;
#define MultiLibLoader MultiLibraryClassLoader*

class MATLABPublisherInterface;
class MATLABSubscriberInterface;
class MATLABSvcServerInterface;
class MATLABSvcClientInterface;

enum ElementType { eMessage, eRequest, eResponse };

class ROS2MsgElementInterfaceFactory{
  public:
    virtual ~ROS2MsgElementInterfaceFactory() = default;
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type)= 0;
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type)= 0;
    virtual std::shared_ptr<MATLABSvcServerInterface> generateSvcServerInterface(){
        std::shared_ptr<MATLABSvcServerInterface> ptr;
        return ptr;
    }
    virtual std::shared_ptr<MATLABSvcClientInterface> generateSvcClientInterface(){
        std::shared_ptr<MATLABSvcClientInterface> ptr;
        return ptr;
    }
};

class MATLABROS2MsgInterfaceBase{
    public:
    virtual ~MATLABROS2MsgInterfaceBase(){}
};

template<class Ros2MessageType>
class MATLABROS2MsgInterface : public MATLABROS2MsgInterfaceBase{
  public:
    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* mCommonObjMap; 
    virtual ~MATLABROS2MsgInterface() {
    }
    
    virtual MDArray_T get_arr(MDFactory_T& factory, const Ros2MessageType* msg, 
            MultiLibLoader loader, size_t size = 1) = 0;

#ifndef FOUNDATION_MATLABDATA_API
    virtual void copy_from_arr(Ros2MessageType* msg, const matlab::data::StructArray& arr,
            MultiLibLoader loader)
#else
    virtual void copy_from_arr(Ros2MessageType* msg, const foundation::matlabdata::StructArray& arr,
            MultiLibLoader loader)
#endif
    final {
        copy_from_struct(msg, arr[0],loader);
    }

#ifndef FOUNDATION_MATLABDATA_API
    virtual void copy_from_struct(Ros2MessageType* msg, const matlab::data::Struct& arr,
            MultiLibLoader loader) = 0;/*{}*/
#endif

    template <typename DependentType>
    inline MATLABROS2MsgInterface<DependentType>*
    getCommonObject(const std::string& className, MultiLibLoader loader) {
        auto mapIt = (*mCommonObjMap).find(className);
        if(mapIt == (*mCommonObjMap).end()){
            auto commonObj = loader->createInstance<MATLABROS2MsgInterface<DependentType>>(className);
            commonObj->mCommonObjMap = mCommonObjMap;
            (*mCommonObjMap)[className] = commonObj;
            return commonObj.get();
        }else{
            return (MATLABROS2MsgInterface<DependentType>*)(mapIt->second.get());
        }
    }
};

#endif // MATLABROSMsgInterface1_H
