// ROS2ServiceTemplates.hpp
// Copyright 2021 The MathWorks, Inc.

#ifndef ROSSERVICETEMPLATES_H
#define ROSSERVICETEMPLATES_H

#include "MATLABROS2MsgInterface.hpp"
#include "MATLABSvcServerInterface.hpp"
#include "MATLABSvcClientInterface.hpp"

/*
    Class : ROS2SvcServerImpl
    Description : This class implements ros2 service server.
*/
template<class RosSvcType, class RosSvcRequestType, class RosSvcResponseType, class RequestCommonType, class ResponseCommonType>
class ROS2SvcServerImpl : public MATLABSvcServerInterface {
    std::shared_ptr<rclcpp::Service<RosSvcType>> mSvc;
    std::shared_ptr<RosSvcResponseType> mResponse;
    RequestCommonType mReqCommonObj;
    ResponseCommonType mRespCommonObj;
  public:
    ROS2SvcServerImpl()
        : MATLABSvcServerInterface() {
    }
    virtual ~ROS2SvcServerImpl() {
    }

    virtual bool setResponseFromMatlab(const matlab::data::StructArray& arr) {
        mRespCommonObj.copy_from_struct(mResponse.get(), arr[0], mMultiLibLoader);
        return true;
    }
	virtual void setCommonObjMap(std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
		mCommonObjMap = commonObjMap;
		mReqCommonObj.mCommonObjMap = mCommonObjMap;
		mRespCommonObj.mCommonObjMap = mCommonObjMap;
	}
    virtual intptr_t createSvcServer(const std::string& svc_name,
                                     rclcpp::Node::SharedPtr theNode,
                                     const rclcpp::QoS& qos,
                                     void* sd,
                                     SendDataToMATLABFunc_T sendDataToMATLABFunc,
                                     intptr_t hSvcServer) {

        auto callback =
            [this, sd, sendDataToMATLABFunc, hSvcServer](
                const std::shared_ptr<rmw_request_id_t> request_header,
                const std::shared_ptr<RosSvcRequestType> request,
                std::shared_ptr<RosSvcResponseType> response) -> void {
            mResponse = response;
            if (sd != NULL) {
                auto outArray =
                    mReqCommonObj.get_arr(mFactory, request.get(), mMultiLibLoader);
                appendAndSendToMATLAB(sd, sendDataToMATLABFunc, outArray, hSvcServer,
                                      request_header);
            }
           
        };

        mSvc = theNode->create_service<RosSvcType>(svc_name, callback, qos.get_rmw_qos_profile());
        return reinterpret_cast<intptr_t>(mSvc.get());
    }
    };

/*
    Class : ROS2SvcClientImpl
    Description : This class implements ros2 service client.
*/
template<class RosSvcType, class RosSvcRequestType, class RosSvcResponseType, class RequestCommonType, class ResponseCommonType, class ServiceResponseFuture>
class ROS2SvcClientImpl : public MATLABSvcClientInterface {
    std::shared_ptr<rclcpp::Client<RosSvcType>> mClient;
    std::map<int64_t, std::shared_ptr<RosSvcRequestType>> mRequestPool;
    std::shared_ptr<RosSvcRequestType> mNextRequest;
    RequestCommonType mReqCommonObj;
    ResponseCommonType mRespCommonObj;
  public:
    RCLCPP_SMART_PTR_DEFINITIONS(ROS2SvcClientImpl)
    ROS2SvcClientImpl()
        : MATLABSvcClientInterface() 
        , mNextRequest(){
    }
    virtual ~ROS2SvcClientImpl() {
    }
	virtual void setCommonObjMap(std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
		mCommonObjMap = commonObjMap;
		mReqCommonObj.mCommonObjMap = mCommonObjMap;
		mRespCommonObj.mCommonObjMap = mCommonObjMap;
	}
    virtual intptr_t createSvcClient(const std::string& srv_name, 
                                    const rclcpp::QoS& qos,
                                    rclcpp::Node::SharedPtr theNode) {

        mClient = theNode->create_client<RosSvcType>(srv_name, qos.get_rmw_qos_profile());

        return reinterpret_cast<intptr_t>(mClient.get());
    }
    virtual bool send_request(void* sd,
                              SendDataToMATLABFunc_T sendDataToMATLABFunc,
                              const intptr_t hSvcClient) {
        mSd = sd;
        mSendDataToMATLABFunc = sendDataToMATLABFunc;
        mHsvcClient = hSvcClient;
                
        // check if service-server is available.
        if(!ifServerAvailable(1)) {
            // If server is not started yet, send error message to MATLAB.
            sendErrorMsgToMATLAB("serviceServerNotAvailableError",
                            "service-server not available. Failed to communicate to service-server.");
            return false;
        }

        auto callback = [this, sd, sendDataToMATLABFunc,
                         hSvcClient](ServiceResponseFuture future) -> void {            
            // After the request is processed, Check if timeout has
            // occurred from MATLAB in between. In-case of timeout, no need to send
            // the response to MATLAB.
            if (sd != NULL && !getNextRequestTimeoutState()) {   
                // fetch and send the response back
                try{
                    auto response = future.get();
                    auto outArray =
                        mRespCommonObj.get_arr(mFactory, response.get(), mMultiLibLoader);
                    appendAndSendToMATLAB(sd, sendDataToMATLABFunc, outArray, hSvcClient, mNextRequestId);
                    
                // If server is not able to send the response,
                // send error message to MATLAB.
                }catch (std::exception& e) {
                    std::string errMsg = std::string(e.what()) + 
                            std::string(", Failed to get response from service-server.");
                    sendErrorMsgToMATLAB("serverCallbackError",errMsg);
                }catch(...){
                    sendErrorMsgToMATLAB("serverCallbackError",
                            "Unknown error. Failed to get response from service-server.");
                }
            }
            
            // After receiving response, notify the worker thread 
            // to process next request.
            setWaitingForResponse(false);
            releaseTheWaitForResponse();
        };
        
        // Just before processing the request, double Check if timeout has
        // occurred from MATLAB or not. In-case of timeout, no need to send the
        // request for processing.
        if (getNextRequestTimeoutState()) {
            return false;
        }
        
        // set the waiting flag as true;
        setWaitingForResponse(true);
        
        mClient->async_send_request(mNextRequest, callback);
        
        // Make the thread wait until the response of the current
        // request is received.
        if(isWaitingForResponse()){
            waitForResponse();
        }
        
        return true;
    }
    virtual bool setNextRequest() {
        if (mRequestPool.size() > 0) {
            auto it = mRequestPool.begin();
            mNextRequestId = it->first;
            mNextRequest = it->second;
            
            // delete from map after fetching the request
            mRequestPool.erase(mNextRequestId);
            // set timeout state false before processing request
            setNextRequestTimeoutState(false);
            return true;
        }
        // if no request left, return false.
        return false;
    }
    virtual int64_t addRequestToPool(matlab::data::StructArray arr) {
        auto req = std::make_shared<RosSvcRequestType>();
        mReqCommonObj.copy_from_struct(req.get(), arr[0], mMultiLibLoader);
        static std::atomic_int64_t requestId(0); 
        ++requestId;
        mRequestPool.insert({requestId,req});
        return requestId;
    }
    virtual bool clientRequestTimeout(int64_t requestId) {
        if (requestId == mNextRequestId) {
            // Check if the request is ongoing request, set the timeout
            // status true so that when response comes, it will be ignored
            // and not sent to MATLAB callback.
            setNextRequestTimeoutState(true);
        } else {
            // If the request is present in request-pool, it has not been
            // processed yet and it is waiting for the earlier request to
            // be processed. As already MATLAB timeout happened, so no need
            // to send to service-server any more and it will be removed.
            if (mRequestPool.find(requestId) != mRequestPool.end()) {
                mRequestPool.erase(requestId);
            } else {
                return false;
            }
        }
        return true;
    }
    
    virtual bool ifServerAvailable(int64_t timeoutMs) {
        try{
            return mClient->wait_for_service(std::chrono::milliseconds(timeoutMs));
        }catch(...){
            return false;
        }
    }
};
#endif // ROSSERVICETEMPLATES_H