// Copyright 2021 The MathWorks, Inc.
#ifndef _MLROSCPP_SVCSERVER_H_
#define _MLROSCPP_SVCSERVER_H_

#include <iostream>
#include <ros/ros.h>
#include "ros_structmsg_conversion.h" // For msg2struct()
#include <mutex>

#define MATLABSvcServer_createSvcServer(obj, mlSvcName, mlSvcSize) \
    obj->createSvcServer(mlSvcName, mlSvcSize)
#define MATLABSvcServer_lock(obj) obj->lock()
#define MATLABSvcServer_unlock(obj) obj->unlock()

template <class ReqMsgType, class RespMsgType, class ReqStructType, class RespStructType>
class MATLABSvcServer {
  public:
    MATLABSvcServer(std::function<void(void)> callback,
                    ReqStructType* reqStructPtr,
                    RespStructType* respStructPtr)
        : callback_{callback}
        , reqStructPtr_{reqStructPtr}
        , respStructPtr_{respStructPtr} {
    }

    /**
     * Create a service server and register it into current ROS network.
     * @param mlSvcName - the name of the service passed from MATLAB
     * @param mlSvcSize - the length of the service name
     */
    void createSvcServer(const char* mlSvcName, size_t mlSvcSize) {
        std::string svcname(mlSvcName, mlSvcSize);
        ros::NodeHandle nh;
        service_ = nh.advertiseService(svcname, &MATLABSvcServer::serviceCallback, this);
    }

    /**
     * Service callback for this service server
     * @param reqMsg - Service request message
     * @param respMsg - Service response message
     */
    bool serviceCallback(ReqMsgType& reqMsg, RespMsgType& respMsg) {
        // Update the request and response message
        mutex_.lock();
        lastReqMsgPtr_ = &reqMsg;
        lastRespMsgPtr_ = &respMsg;
        msg2struct(reqStructPtr_, lastReqMsgPtr_);
        msg2struct(respStructPtr_, lastRespMsgPtr_);
        mutex_.unlock();
        // Call the callback function specified in MLCoderSvcServer
        callback_();
        mutex_.lock();
        struct2msg(&respMsg, respStructPtr_);
        mutex_.unlock();
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

  private:
    ReqStructType* reqStructPtr_;
    RespStructType* respStructPtr_;
    ros::ServiceServer service_;
    const ReqMsgType* lastReqMsgPtr_;
    const RespMsgType* lastRespMsgPtr_;
    std::mutex mutex_;
    std::function<void(void)> callback_;
};

#endif
