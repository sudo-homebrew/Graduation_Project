// Copyright 2021 The MathWorks, Inc.
#ifndef _MLROS2_SUB_H_
#define _MLROS2_SUB_H_

#include <iostream>
#include <memory>
#include <string>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "mlros2_qos.h"
#include "ros2_structmsg_conversion.h" // For msg2struct()

#define MATLABROS2Subscriber_lock(obj) obj->lock()
#define MATLABROS2Subscriber_unlock(obj) obj->unlock()
#define MATLABROS2Subscriber_createSubscriber(obj, theNode, mlTopic, mlTopicSize, qos_profile) \
    obj->createSubscriber(theNode, mlTopic, mlTopicSize, qos_profile)


template <class MsgType, class StructType>
class MATLABROS2Subscriber {
    std::function<void(void)> MATLABCallback_;
    StructType* structPtr_;
    std::shared_ptr<rclcpp::Subscription<MsgType>> mSub_;
    std::shared_ptr<MsgType> lastMsgPtr_;
    std::mutex mtx_;

  public:
    RCLCPP_SMART_PTR_DEFINITIONS(MATLABROS2Subscriber)
    MATLABROS2Subscriber(StructType* structPtr, std::function<void(void)> callback)
        : structPtr_{structPtr}
        , MATLABCallback_{callback} {
    }

    /**
     * Method to create a ROS 2 Subscriber.
     * @param theNode Shared pointer to the ROS 2 Node Handle
     * @param mlTopic Topic on which a message is to be published.
     * @param mlTopicSize size to handle string inputs.
     * @param qos_profile qos settings.
     */
    void createSubscriber(rclcpp::Node::SharedPtr theNode,
                          const char* mlTopic,
                          size_t mlTopicSize,
                          const rmw_qos_profile_t& qos_profile = rmw_qos_profile_default) {
        std::string topic(mlTopic, mlTopicSize);
        auto subscriberCallback = [this](const std::shared_ptr<MsgType> msgPtr) {
            std::lock_guard<std::mutex> lockMsg(mtx_);
            lastMsgPtr_ = msgPtr; // copy the shared_ptr
            msg2struct(structPtr_, *lastMsgPtr_.get());
            MATLABCallback_(); // Call MATLAB callback
        };
        // Subscribe to given topic and set callback
        mSub_ = theNode->create_subscription<MsgType>(topic, getQOSSettingsFromRMW(qos_profile),
                                                      subscriberCallback);
    }

    void lock() {
        mtx_.lock();
    }

    void unlock() {
        mtx_.unlock();
    }
};

/**
 * Function to get status text.
 */
extern void getStatusText(bool status, char* mlStatusText);
#endif
