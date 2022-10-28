// Copyright 2021 The MathWorks, Inc.
#ifndef _MLROS2_SVCCLIENT_H_
#define _MLROS2_SVCCLIENT_H_

#include <iostream>
#include <memory>
#include <string>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "mlros2_qos.h"
#include "ros2_structmsg_conversion.h" // For msg2struct()
#include <functional>                  // For std::function
#include <future>                      // For std::future
#include <chrono>                      // For std::chrono

#define MATLABROS2SvcClient_lock(obj) obj->lock()
#define MATLABROS2SvcClient_unlock(obj) obj->unlock()
#define MATLABROS2SvcClient_createSvcClient(obj, theNode, mlSvcName, mlSvcSize, qos_profile) \
    obj->createSvcClient(theNode, mlSvcName, mlSvcSize, qos_profile)
#define MATLABROS2SvcClient_callService(obj, callTimeoutMS, statusIndex) \
    obj->callService(callTimeoutMS, statusIndex)
#define MATLABROS2SvcClient_waitForService(obj, waitTimeout, status) \
    obj->waitForService(waitTimeout, status)
#define MATLABROS2SvcClient_isServerAvailable(obj, status) obj->isServerAvailable(status)

template <class SvcType,
          class ReqMsgType,
          class RespMsgType,
          class ReqStructType,
          class RespStructType>
class MATLABROS2SvcClient {
  public:
    RCLCPP_SMART_PTR_DEFINITIONS(MATLABROS2SvcClient)
    MATLABROS2SvcClient(ReqStructType* reqStructPtr, RespStructType* respStructPtr)
        : reqStructPtr_{reqStructPtr}
        , respStructPtr_{respStructPtr} {
    }

    /**
     * Create a service client and register it into current ROS 2 network.
     * @param theNode - pointer to ROS 2 Node
     * @param mlSvcName - the name of the service passed from MATLAB
     * @param mlSvcSize - the length of the service name
     * @param qos_profile - QoS profile passed from MATLAB
     */
    void createSvcClient(rclcpp::Node::SharedPtr theNode,
                         const char* mlSvcName,
                         size_t mlSvcSize,
                         const rmw_qos_profile_t& qos_profile = rmw_qos_profile_default) {
        std::string svcname(mlSvcName, mlSvcSize);
        theNode_ = theNode;
        client_ = theNode_->create_client<SvcType>(svcname, qos_profile);
    }

    /**
     * Send out a request from this service client to the server.
     * @param callTimeout - timeout for calling the service
     * @param state_ - double pointer to track the calling status
     */
    void callService(int callTimeout, double* state_) {
        mutex_.lock();
        struct2msg(*reqMsgPtr_.get(), reqStructPtr_);
        auto result = client_->async_send_request(reqMsgPtr_);

        std::future_status reqStatus;
        if (callTimeout > 0) {
            // User specified timeout
            reqStatus = result.wait_for(std::chrono::milliseconds(callTimeout));
            if (reqStatus == std::future_status::ready) {
                // Received response before timeout
                *state_ = 0;
                msg2struct(respStructPtr_, *result.get());
            } else {
                // Timeout
                *state_ = 1;
            }
        } else {
            // Use default timeout, which is Inf.
            do {
                reqStatus = result.wait_for(std::chrono::seconds(1));
            } while (reqStatus != std::future_status::ready);
            *state_ = 0;
            msg2struct(respStructPtr_, *result.get());
        }
        mutex_.unlock();
    }

    /**
     * One time check on whether server is available.
     * @param status - boolean pointer to track whether server is available
     */
    void isServerAvailable(bool* status) {
        *status = client_->wait_for_service(std::chrono::milliseconds(1));
    }

    /**
     * Wait for server to be available.
     * @param timeout - wait timeout carried from MATLAB
     * @param status - boolean pointer to track server status
     */
    void waitForService(int timeout, bool* status) {
        if (timeout > 0) {
            *status = client_->wait_for_service(std::chrono::milliseconds(timeout));
        } else {
            *status = client_->wait_for_service();
        }
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

  private:
    std::shared_ptr<ReqMsgType> reqMsgPtr_ = std::make_shared<ReqMsgType>();
    std::shared_ptr<RespMsgType> respMsgPtr_ = std::make_shared<RespMsgType>();
    ReqStructType* reqStructPtr_;
    RespStructType* respStructPtr_;
    std::shared_ptr<rclcpp::Client<SvcType>> client_;
    std::mutex mutex_;
    std::shared_ptr<rclcpp::Node> theNode_;
};

#endif
