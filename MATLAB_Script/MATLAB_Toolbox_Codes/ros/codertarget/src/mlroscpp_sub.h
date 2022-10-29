// Copyright 2020-2021 The MathWorks, Inc.
#ifndef _MLROSCPP_SUB_H_
#define _MLROSCPP_SUB_H_

#include <iostream>
#include <mutex>
#include <functional> // For std::function
#include <ros/ros.h>
#include "ros_structmsg_conversion.h" // For msg2struct()

#define MATLABSUBSCRIBER_lock(obj)   obj->lock()
#define MATLABSUBSCRIBER_unlock(obj) obj->unlock()
#define MATLABSUBSCRIBER_createSubscriber(obj,mlTopic,mlTopicSize,queueSize) obj->createSubscriber(mlTopic,mlTopicSize,queueSize) 

template <class MsgType, class StructType>
class MATLABSubscriber {
public:
    MATLABSubscriber(StructType* structPtr, std::function<void(void)> callback): 
        structPtr_{structPtr}, MATLABCallback_{callback} {}
            
    void createSubscriber(const char* mlTopic, size_t mlTopicSize, uint32_t queueSize) {
        std::string topic(mlTopic, mlTopicSize);
        ros::NodeHandle nh;
        
        // Subscribe to given topic and set callback
        sub_ = nh.subscribe(topic, queueSize, &MATLABSubscriber::subscriberCallback, this, ros::TransportHints().tcpNoDelay(true));
    }
            
    void subscriberCallback(const boost::shared_ptr<MsgType const>& msgPtr) {
        mutex_.lock();
        lastMsgPtr_ = msgPtr; // copy shared_ptr 
        msg2struct(structPtr_, lastMsgPtr_.get()); 
        // Call MATLAB callback function. NOTE: this call accesses structPtr_ 
        // hence must be protected by a mutex 
        MATLABCallback_();      
        mutex_.unlock();
    }

    void lock() {
        mutex_.lock();
    }

    void unlock() {
        mutex_.unlock();
    }
            
private:
    std::function<void(void)> MATLABCallback_;
    StructType *structPtr_;
    ros::Subscriber sub_;
    boost::shared_ptr<MsgType const> lastMsgPtr_;
    std::mutex mutex_;
};


/**
* Function to get status text.
*/
extern void getStatusText(bool status, char* mlStatusText);

#endif
