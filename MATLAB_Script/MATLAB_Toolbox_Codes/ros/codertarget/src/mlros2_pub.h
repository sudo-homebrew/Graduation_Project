// Copyright 2021 The MathWorks, Inc.
#ifndef _MLROS2_PUB_H
#define _MLROS2_PUB_H

#include <iostream>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "mlros2_qos.h"
#include "ros2_structmsg_conversion.h" // For struct2msg()
#include <type_traits>

#define MATLABROS2Publisher_createPublisher(obj,theNode,mlTopic,mlTopicSize,qos_profile) \
    obj->createPublisher(theNode,mlTopic,mlTopicSize,qos_profile)
#define MATLABROS2Publisher_publish(obj,structPtr) obj->publish(structPtr)

template <class MsgType, class StructType>
class MATLABROS2Publisher {
    std::shared_ptr<rclcpp::Publisher<MsgType>> mPub_;
    std::shared_ptr<MsgType> msgPtr_;
  public:
    RCLCPP_SMART_PTR_DEFINITIONS(MATLABROS2Publisher)
    MATLABROS2Publisher()
        : msgPtr_(new MsgType) {
    }

    /**
    * Method to create a ROS 2 publisher.
    * @param theNode Shared pointer to the ROS 2 Node Handle
    * @param mlTopic Topic on which a message is to be published.
    * @param mlTopicSize size to handle string inputs.
    * @param qos_profile qos settings.
    */
    void createPublisher(rclcpp::Node::SharedPtr theNode,
                         const char* mlTopic,
                         size_t mlTopicSize,
                         const rmw_qos_profile_t& qos_profile = rmw_qos_profile_default) {
         std::string topic(mlTopic, mlTopicSize);
         // Create a subscription to the topic which can be matched with one or more compatible ROS
         // publishers.
         // Note that not all publishers on the same topic with the same type will be compatible:
         // they must have compatible Quality of Service policies.
         mPub_ = theNode->create_publisher<MsgType>(topic, getQOSSettingsFromRMW(qos_profile));
    }

     /**
     * Method to publish a ROS 2 message.
     * @param msgStruct ROS 2 message structure
     */
     void publish(const StructType *msgStructPtr) {
         struct2msg(*msgPtr_.get(), msgStructPtr);
         mPub_->publish(*msgPtr_);
     }
};

#endif
