// Copyright 2020-2021 The MathWorks, Inc.
#ifndef _MLROSCPP_PUB_H
#define _MLROSCPP_PUB_H

#include <iostream>
#include <ros/ros.h>
#include "ros_structmsg_conversion.h" // For struct2msg()


#define MATLABPUBLISHER_createPublisher(obj,mlTopic,mlTopicSize,queueSize,latch) obj->createPublisher(mlTopic, mlTopicSize, queueSize, latch)
#define MATLABPUBLISHER_publish(obj,msgStructPtr) obj->publish(msgStructPtr)
#define MATLABPUBLISHER_getNumSubscribers(obj) obj->getNumSubscribers()


template <class MsgType, class StructType>
class MATLABPublisher {
  public:
    MATLABPublisher() : msgPtr_(new MsgType) {}

    void createPublisher(const char* mlTopic, size_t mlTopicSize, uint32_t queueSize, bool latch) {
        std::string topic(mlTopic, mlTopicSize);
        ros::NodeHandle nh;

        pub_ = nh.advertise<MsgType>(topic, queueSize, latch);
    }

    void publish(const StructType* msgStructPtr) {
        struct2msg(msgPtr_.get(), msgStructPtr);
        pub_.publish(*msgPtr_);
    }
    
    uint32_t getNumSubscribers() {
        return pub_.getNumSubscribers();
    }

  private:
    ros::Publisher pub_;
    boost::shared_ptr<MsgType> msgPtr_;
};

#endif
