// ROSServiceTemplates.hpp
// Copyright 2020-2021 The MathWorks, Inc.

#ifndef ROSSERVICETEMPLATES_H
#define ROSSERVICETEMPLATES_H

#include "MATLABROSMsgInterface.hpp"
#include "MATLABSvcServerInterface.hpp"
#include "MATLABSvcClientInterface.hpp"

template<class RosSvcRequestType, class RosSvcResponseType, class RequestCommonType, class ResponseCommonType>
class ROSSvcServerImpl : public MATLABSvcServerInterface {
    RosSvcResponseType *mResponse;
    void* mSd;
    SendDataToMATLABFunc_T mSendDataToMATLABFunc;
    intptr_t mHSvcServer;
    RequestCommonType mReqCommonObj;
    ResponseCommonType mRespCommonObj;
    bool callback(RosSvcRequestType& req,
                  RosSvcResponseType& res) {
        mResponse = &res;
        auto outArray = mReqCommonObj.get_arr(mFactory, &req, mMultiLibLoader);
        if (mSd != NULL) {
            appendAndSendToMATLAB(mSd, mSendDataToMATLABFunc, outArray, mHSvcServer);
        }
        // After receiving the response, check if any error occurred
        // and in case of error, return false to service-client.
        if (getCallbackError()) {
            return false;
        }
        // If everything is fine, send the received response.
        return true;
    }
  public:
    ROSSvcServerImpl()
        : MATLABSvcServerInterface()
        , mResponse(NULL)
        , mSd(NULL)
        , mSendDataToMATLABFunc()
        , mHSvcServer(0) {
    }
    virtual ~ROSSvcServerImpl() {
    }
    virtual void setCommonObjMap(std::map<std::string,boost::shared_ptr<MATLABROSMsgInterfaceBase>>* commonObjMap){
        mCommonObjMap = commonObjMap;
        mReqCommonObj.mCommonObjMap = mCommonObjMap;
        mRespCommonObj.mCommonObjMap = mCommonObjMap;
    }
    virtual bool setResponseFromMatlab(const matlab::data::StructArray arr) {
        mRespCommonObj.copy_from_struct(mResponse, arr[0], mMultiLibLoader);
        return true;
    }
    virtual intptr_t createSvcServer(const std::string& svc_name,
                                     std::shared_ptr<ros::NodeHandle> theNode,
                                     void* sd,
                                     SendDataToMATLABFunc_T sendDataToMATLABFunc,
                                     intptr_t hSvcServer) {
        mSd = sd;
        mSendDataToMATLABFunc = sendDataToMATLABFunc;
        mHSvcServer = hSvcServer;
        mSvc = std::make_shared<ros::ServiceServer>(theNode->advertiseService(
            svc_name, &ROSSvcServerImpl::callback, this));
        return reinterpret_cast<intptr_t>(mSvc.get());
    }
};

template<class RosSvcType, class RosSvcRequestType, class RosSvcResponseType, class RequestCommonType, class ResponseCommonType>
class ROSSvcClientImpl : public MATLABSvcClientInterface {
    // mRequestPool is moved from SvcClientData to here so that we can throw the user field-error
    // exception in MATLAB console.
    std::map<int64_t, RosSvcRequestType> mRequestPool;
    RosSvcRequestType mNextRequest;
    RequestCommonType mReqCommonObj;
    ResponseCommonType mRespCommonObj;
  public:
    ROSSvcClientImpl()
        : MATLABSvcClientInterface()
        , mNextRequest() {
    }
    virtual ~ROSSvcClientImpl() {
    }
    virtual void setCommonObjMap(std::map<std::string,boost::shared_ptr<MATLABROSMsgInterfaceBase>>* commonObjMap){
        mCommonObjMap = commonObjMap;
        mReqCommonObj.mCommonObjMap = mCommonObjMap;
        mRespCommonObj.mCommonObjMap = mCommonObjMap;
    }
    virtual intptr_t createSvcClient(const std::string& srv_name,
                                     std::shared_ptr<ros::NodeHandle> theNode) {
        mClient = std::make_shared<ros::ServiceClient>(
            theNode->serviceClient<RosSvcType>(srv_name));
        return reinterpret_cast<intptr_t>(mClient.get());
    }
    virtual bool send_request(void* sd,
                              SendDataToMATLABFunc_T sendDataToMATLABFunc,
                              const intptr_t hSvcClient) {
        RosSvcType svc;
        svc.request = mNextRequest;
        // Just before processing the request, double Check if timeout has
        // occurred from MATLAB or not. In-case of timeout, no need to send the
        // request for processing.
        if (getNextRequestTimeoutState()) {
            return false;
        }
        // send the request for processing
        auto ret = mClient->call(svc);
        // After the request is processed, Check if timeout has
        // occurred from MATLAB in between. In-case of timeout, no need to send
        // the response to MATLAB.
        if (getNextRequestTimeoutState()) {
            return false;
        }
        if (ret) {
            // send the result back
            auto outArray =
                mRespCommonObj.get_arr(mFactory, &svc.response, mMultiLibLoader);
            if (sd != NULL) {
                appendAndSendToMATLAB(sd, sendDataToMATLABFunc, outArray, hSvcClient,
                                      mNextRequestId);
            } else {
                return false;
            }
        } else {
            // If server is not able to send the response,
            // send error message to MATLAB.
            auto outArray = mFactory.createStructArray({1, 1}, {"serverCallbackError"});
            outArray[0]["serverCallbackError"] =
                mFactory.createCharArray("Failed to get response from service-server.");
            if (sd != NULL) {
                appendAndSendToMATLAB(sd, sendDataToMATLABFunc, outArray, hSvcClient,
                                      mNextRequestId);
            }
            return false;
        }
        return true;
    }
    virtual bool setNextRequest() {
        if (mRequestPool.size() > 0) {
            auto it = mRequestPool.begin();
            mNextRequestId = it->first;
            mNextRequest = it->second;
            std::cout << "\nNext request-id : " << mNextRequestId;
            // delete from map after fetching the request
            mRequestPool.erase(mNextRequestId);
            // set timeout state false before processing request
            setNextRequestTimeoutState(false);
            return true;
        }
        // if no request left, return false.
        return false;
    }
    virtual bool addRequestToPool(matlab::data::StructArray arr) {
        RosSvcRequestType req;
        mReqCommonObj.copy_from_struct(&req, arr[0], mMultiLibLoader);
        mRequestPool.insert({get_requestId_from_arr(arr),req});
        return true;
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
};
#endif // ROSSERVICETEMPLATES_H