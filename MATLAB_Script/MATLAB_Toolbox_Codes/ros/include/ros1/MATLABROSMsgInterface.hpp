// MATLABROSMsgInterface.hpp
// Copyright 2020-2021 The MathWorks, Inc.

#ifndef MATLABROSMSGINTERFACE1_H
#define MATLABROSMSGINTERFACE1_H

#include "MATLABInterfaceCommon.hpp"

class MATLABPublisherInterface;
class MATLABSubscriberInterface;
class MATLABSvcServerInterface;
class MATLABSvcClientInterface;
class MATLABActClientInterface;
class MATLABActServerInterface;
class MATLABRosbagWriterInterface;

enum ElementType { eMessage, eRequest, eResponse, eGoal, eFeedback, eResult };

class ROSMsgElementInterfaceFactory{
  public:
    virtual ~ROSMsgElementInterfaceFactory() = default;
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type)= 0;
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type)= 0;
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType /*type*/){
        std::shared_ptr<MATLABRosbagWriterInterface> ptr;
        return ptr;
    }
    virtual std::shared_ptr<MATLABSvcServerInterface> generateSvcServerInterface(){
        std::shared_ptr<MATLABSvcServerInterface> ptr;
        return ptr;
    }
    virtual std::shared_ptr<MATLABSvcClientInterface> generateSvcClientInterface(){
        std::shared_ptr<MATLABSvcClientInterface> ptr;
        return ptr;
    }
    virtual std::shared_ptr<MATLABActClientInterface> generateActClientInterface(){
        std::shared_ptr<MATLABActClientInterface> ptr;
        return ptr;
    }
    virtual std::shared_ptr<MATLABActServerInterface> generateActServerInterface(){
        std::shared_ptr<MATLABActServerInterface> ptr;
        return ptr;
    }
};

class MATLABROSMsgInterfaceBase{
    public:
    virtual ~MATLABROSMsgInterfaceBase(){}
};
template<class RosMessageType>
class MATLABROSMsgInterface : public MATLABROSMsgInterfaceBase{
  public:
    std::map<std::string,boost::shared_ptr<MATLABROSMsgInterfaceBase>>* mCommonObjMap; 
    virtual ~MATLABROSMsgInterface() {
    }
    
    virtual MDArray_T get_arr(MDFactory_T& factory, const RosMessageType* msg, 
            MultiLibLoader loader, size_t size = 1) = 0;

#ifndef FOUNDATION_MATLABDATA_API
    virtual void copy_from_struct(RosMessageType* msg, const matlab::data::Struct& arr,
            MultiLibLoader loader) = 0;
#endif

    template <typename DependentType>
    inline MATLABROSMsgInterface<DependentType>*
    getCommonObject(const std::string& className, MultiLibLoader loader) {
        auto mapIt = (*mCommonObjMap).find(className);
        if(mapIt == (*mCommonObjMap).end()){
            auto commonObj = loader->createInstance<MATLABROSMsgInterface<DependentType>>(className);
            commonObj->mCommonObjMap = mCommonObjMap;
            (*mCommonObjMap)[className] = commonObj;
            return commonObj.get();
        }else{
            return (MATLABROSMsgInterface<DependentType>*)(mapIt->second.get());
        }
    }
};

#endif // MATLABROSMsgInterface1_H
