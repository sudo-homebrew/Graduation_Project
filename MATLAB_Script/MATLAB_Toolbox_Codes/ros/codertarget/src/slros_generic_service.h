/* Copyright 2018 The MathWorks, Inc. */

#ifndef _SLROS_GENERIC_SERVICE_H_
#define _SLROS_GENERIC_SERVICE_H_

#include <iostream>
#include <ros/ros.h>

extern ros::NodeHandle*
    SLROSNodePtr; ///< The global node handle that is used by all ROS entities in the model

/**
 * Class for ROS service client in C++.
 *
 * This class is used by code generated from the Simulink ROS
 * "Call Service" blocks and is templatized by the ROS service type and
 * the Simulink bus types for the request and response messages.
 */
template <class SrvType, class ReqBusType, class RespBusType>
class SimulinkServiceCaller {
  public:
    SimulinkServiceCaller();
    bool getIsCreated();
    void setIsCreated(bool isCreated);
    uint8_T createServiceCaller(const std::string& serviceName,
                                bool persistent,
                                double connectionTimeout);
    uint8_T call(ReqBusType* reqBusPtr, RespBusType* respBusPtr);

  private:
    ros::ServiceClient _serviceCaller; ///< The ROS service caller object
    SrvType _srv;                      ///< The service type
    bool _isCreated;                   ///< Indicates if the service client has been created
    boost::mutex _mtx;
};

/**
 * Standard constructor
 */
template <class SrvType, class ReqBusType, class RespBusType>
SimulinkServiceCaller<SrvType, ReqBusType, RespBusType>::SimulinkServiceCaller()
    : _isCreated(false) {
}

/**
 * Get the value of the _isCreated member
 * @return Has the service client been created?
 */
template <class SrvType, class ReqBusType, class RespBusType>
bool SimulinkServiceCaller<SrvType, ReqBusType, RespBusType>::getIsCreated() {
    return _isCreated;
}

/**
 * Set the value of the _isCreated member
 * @param[in] isCreated Has the service client been created?
 */
template <class SrvType, class ReqBusType, class RespBusType>
void SimulinkServiceCaller<SrvType, ReqBusType, RespBusType>::setIsCreated(bool isCreated) {
    _isCreated = isCreated;
}

/**
 * Create a ROS service client and connect to service
 *
 * @param[in] serviceName The name of the service to call
 * @param[in] isPersistent Should service connection be persistent?
 * @param[in] connectionTimeout Connection timeout (in seconds)
 * @retval 0 Successful creation of service client
 * @retval 1 The client could not be created within the specified timeout
 */
template <class SrvType, class ReqBusType, class RespBusType>
uint8_T SimulinkServiceCaller<SrvType, ReqBusType, RespBusType>::createServiceCaller(
    const std::string& serviceName,
    bool isPersistent,
    double connectionTimeout) {
    uint8_T errorCode = 0;
    bool success = ros::service::waitForService(serviceName, ros::Duration(connectionTimeout));
    if (success) {
        // The service is available. Connect to it.
        _serviceCaller = SLROSNodePtr->serviceClient<SrvType>(serviceName, isPersistent);
        _isCreated = true;
    } else {
        // The service has not become available within the maximum connection timeout
        errorCode = 1;
    }

    return errorCode;
}


/**
 * Call the service and receive a response
 *
 * @param reqBusPtr[in] Pointer to the bus structure for the request message
 * @param respBusPtr[out] Pointer to the bus structure for the response message
 * @retval 0 The call to the service succeeded and we have a valid response
 * @retval 3 Something bad happened.
 */
template <class SrvType, class ReqBusType, class RespBusType>
uint8_T SimulinkServiceCaller<SrvType, ReqBusType, RespBusType>::call(ReqBusType* reqBusPtr,
                                                                      RespBusType* respBusPtr) {
    boost::lock_guard<boost::mutex> lockSrv(_mtx);
    // Prepare the request message
    convertFromBus(&_srv.request, reqBusPtr);

    // Call the service. The service client object will handle the persistence
    // setting accordingly.
    if (!_serviceCaller.call(_srv)) {
        // Return if the call did not succeed
        return 2;
    }

    // We received a response. Convert it to a bus and return.
    convertToBus(respBusPtr, &_srv.response);
    return 0;
}

#endif
