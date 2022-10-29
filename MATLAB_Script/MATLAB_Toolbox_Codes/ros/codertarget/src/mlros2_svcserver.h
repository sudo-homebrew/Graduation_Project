// Copyright 2021 The MathWorks, Inc.
#ifndef _MLROS2_SVCSERVER_H_
#define _MLROS2_SVCSERVER_H_

#include <iostream>
#include <memory>
#include <string>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "mlros2_qos.h"
#include "ros2_structmsg_conversion.h" // For msg2struct()
#include <functional>                  // For std::function

#define MATLABROS2SvcServer_lock(obj) obj->lock()
#define MATLABROS2SvcServer_unlock(obj) obj->unlock()
#define MATLABROS2SvcServer_createSvcServer(obj, theNode, mlSvcName, mlSvcSize, qos_profile) \
    obj->createSvcServer(theNode, mlSvcName, mlSvcSize, qos_profile)

template <class SvcType,
          class ReqMsgType,
          class RespMsgType,
          class ReqStructType,
          class RespStructType>
class MATLABROS2SvcServer {
  public:
    RCLCPP_SMART_PTR_DEFINITIONS(MATLABROS2SvcServer)
    MATLABROS2SvcServer(ReqStructType* reqStructPtr,
                        RespStructType* respStructPtr,
                        std::function<void(void)> callback)
        : reqStructPtr_{reqStructPtr}
        , respStructPtr_{respStructPtr}
        , MATLABCallback_{callback} {
    }

    /**
     * Create a service server and register it into current ROS 2 network.
     * @param theNode - pointer to ROS 2 Node
     * @param mlSvcName - the name of the service passed from MATLAB
     * @param mlSvcSize - the length of the service name
     * @param qos_profile - QoS profile passed from MATLAB
     */
    void createSvcServer(rclcpp::Node::SharedPtr theNode,
                         const char* mlSvcName,
                         size_t mlSvcSize,
                         const rmw_qos_profile_t& qos_profile = rmw_qos_profile_default) {
        std::string svcname(mlSvcName, mlSvcSize);
        auto serviceCallback = [this](const std::shared_ptr<ReqMsgType> reqMsgPtr,
                                      std::shared_ptr<RespMsgType> respMsgPtr) {
            mutex_.lock();
            msg2struct(reqStructPtr_, *reqMsgPtr.get());
            msg2struct(respStructPtr_, *respMsgPtr.get());
            MATLABCallback_();
            struct2msg(*respMsgPtr.get(), respStructPtr_);
            mutex_.unlock();
        };
        service_ = theNode->create_service<SvcType>(svcname, serviceCallback, qos_profile);
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
    std::shared_ptr<rclcpp::Service<SvcType>> service_;
    std::mutex mutex_;
    std::function<void(void)> MATLABCallback_;
};

#endif
