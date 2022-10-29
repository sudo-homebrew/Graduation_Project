// Copyright 2021 The MathWorks, Inc.
#ifndef _MLROSCPP_SVCCLIENT_H_
#define _MLROSCPP_SVCCLIENT_H_

#include <iostream>
#include <ros/ros.h>
#include <future>                     // For std::future
#include <chrono>                     // For std::chrono
#include <functional>                 // For std::function
#include "ros_structmsg_conversion.h" // For msg2struct()
#include <mutex>

#define MATLABSvcClient_lock(obj) obj->lock()
#define MATLABSvcClient_unlock(obj) obj->unlock()
#define MATLABSvcClient_createSvcClient(obj, mlSvcName, mlSvcSize) \
    obj->createSvcClient(mlSvcName, mlSvcSize)
#define MATLABSvcClient_mlExists(obj) obj->mlExists()
#define MATLABSvcClient_mlWaitForExistence(obj, timeout, status) \
    obj->mlWaitForExistence(timeout, status)
#define MATLABSvcClient_callService(obj, callTimeout, state_) obj->callService(callTimeout, state_)


template <class SvcType,
          class ReqMsgType,
          class RespMsgType,
          class ReqStructType,
          class RespStructType>
class MATLABSvcClient {
  private:
    ros::ServiceClient client_;
    ReqMsgType reqMsgPtr_;
    RespMsgType respMsgPtr_;
    ReqStructType* reqStructPtr_;
    RespStructType* respStructPtr_;
    double callState_;
    std::mutex mutex_;

  public:
    MATLABSvcClient(ReqStructType* reqStructPtr, RespStructType* respStructPtr)
        : reqStructPtr_(reqStructPtr)
        , respStructPtr_(respStructPtr) {
    }

    /**
     * Create a service client and register it into current ROS network.
     * @param mlSvcName - the name of the service passed from MATLAB
     * @param mlSvcSize - the length of the service name
     */
    void createSvcClient(const char* mlSvcName, size_t mlSvcSize) {
        std::string svcname(mlSvcName, mlSvcSize);
        ros::NodeHandle nh;
        client_ = nh.serviceClient<SvcType>(svcname);
    }

    /**
     * Return whether the service is up and available.
     */
    bool mlExists() {
        return client_.exists();
    }

    /**
     * Waits for the service server to connect to this client.
     * @param timeout - user specified timeout for waiting.
     * @param status - bool pointer to indicate existence.
     */
    void mlWaitForExistence(double timeout, bool* status) {
        if (timeout > 0) {
            *status = client_.waitForExistence(ros::Duration(timeout));
        } else {
            *status = client_.waitForExistence();
        }
    }

    /**
     * Send out a request from this service client to the server.
     * @param callTimeout - timeout for calling the service
     * @param state_ - double pointer to track the calling status
     */
    void callService(int callTimeout, double* state_) {

        mutex_.lock();
        struct2msg(&reqMsgPtr_, reqStructPtr_);
        struct2msg(&respMsgPtr_, respStructPtr_);

        if (callTimeout > 0) {
            std::future<bool> fut = std::async(&MATLABSvcClient::callServiceImpl, this);
            if (fut.wait_for(std::chrono::milliseconds(callTimeout)) != std::future_status::ready) {
                callState_ = 1;
                *state_ = callState_;
                return;
            }
        } else {
            callState_ = (client_.call(reqMsgPtr_, respMsgPtr_)) ? 0 : 2;
        }
        *state_ = callState_;
        msg2struct(reqStructPtr_, &reqMsgPtr_);
        msg2struct(respStructPtr_, &respMsgPtr_);
        mutex_.unlock();
        return;
    }

    /**
     * Call service implementation for use of std::future and std::async
     */
    bool callServiceImpl() {
        callState_ = (client_.call(reqMsgPtr_, respMsgPtr_)) ? 0 : 2;
        return true;
    }

    /**
     * Mutex lock to avoid read and write conflict on messages
     */
    void lock() {
        mutex_.lock();
    }

    /**
     * Mutex unlock to release mutex lock
     */
    void unlock() {
        mutex_.unlock();
    }
};

#endif
