// ROSPubSubTemplates.hpp
// Copyright 2020-2021 The MathWorks, Inc.

#ifndef ROSPUBSUBTEMPLATES_H
#define ROSPUBSUBTEMPLATES_H

#include "MATLABPublisherInterface.hpp"
#include "MATLABSubscriberInterface.hpp"
#include "MATLABROSMsgInterface.hpp"

template<class RosMessageType, class CommonType>
class ROSPublisherImpl : public MATLABPublisherInterface {
    
  std::shared_ptr<ros::Publisher> mPub;
  CommonType mCommonObj;
  public:
            
    ROSPublisherImpl()
        : MATLABPublisherInterface(){
            
    }
    virtual void setCommonObjMap(std::map<std::string,boost::shared_ptr<MATLABROSMsgInterfaceBase>>* commonObjMap){
        mCommonObjMap = commonObjMap;
        mCommonObj.mCommonObjMap = mCommonObjMap;
    }
    virtual ~ROSPublisherImpl() {
    }
    virtual intptr_t createPublisher(const std::string& topic_name,
                                     bool latching,
                                     std::shared_ptr<ros::NodeHandle> n) {
        // Create a subscription to the topic which can be matched with one or more compatible ROS
        // publishers.
        // Note that not all publishers on the same topic with the same type will be compatible:
        // they must have compatible Quality of Service policies.
        mPub = std::make_shared<ros::Publisher>(n->advertise<RosMessageType>(topic_name, 1000,latching));
        return reinterpret_cast<intptr_t>(mPub.get());
    }
    virtual bool publish(const matlab::data::StructArray& arr) {
        auto msg = boost::make_shared<RosMessageType>();
        mCommonObj.copy_from_struct(msg.get(), arr[0], mMultiLibLoader);
        mPub->publish(msg);
        return true;
    }
};

template<class RosMessageType, class RosCallbackTypePtr, class CommonType>
class ROSSubscriberImpl : public MATLABSubscriberInterface {
    std::shared_ptr<ros::Subscriber> mSub;
    void* mSd;
    SendDataToMATLABFunc_T mSendDataToMATLABFunc;
    CommonType mCommonObj;
  public:
    // Create a callback function for when messages are received.
    // Variations of this function also exist using, for example UniquePtr 
    // for zero-copy transport.
    void callback(const RosCallbackTypePtr& msg){
      if(mSd){
        auto outArray = mCommonObj.get_arr(mFactory, msg.get(), mMultiLibLoader);
        appendAndSendToMATLAB(mSd, mSendDataToMATLABFunc, outArray);
      }
    }
    virtual void setCommonObjMap(std::map<std::string,boost::shared_ptr<MATLABROSMsgInterfaceBase>>* commonObjMap){
        mCommonObjMap = commonObjMap;
        mCommonObj.mCommonObjMap = mCommonObjMap;
    }
    ROSSubscriberImpl()
        : MATLABSubscriberInterface(){
    }
    virtual ~ROSSubscriberImpl() {
    }
    virtual intptr_t createSubscription(const std::string& topic_name,
                                        std::shared_ptr<ros::NodeHandle> n,
                                        void* sd,
                                        SendDataToMATLABFunc_T sendDataToMATLABFunc,
                                        uint32_t buffer_size) {
        mSd = sd;
        mSendDataToMATLABFunc = sendDataToMATLABFunc;
        mSub = std::make_shared<ros::Subscriber>(
                n->subscribe<RosMessageType>(
                            topic_name, 
                            buffer_size, 
                            &ROSSubscriberImpl::callback, 
                            this,
                            ros::TransportHints().reliable().tcpNoDelay()));
        return true;
    }
};
#endif // ROSPUBSUBTEMPLATES_H