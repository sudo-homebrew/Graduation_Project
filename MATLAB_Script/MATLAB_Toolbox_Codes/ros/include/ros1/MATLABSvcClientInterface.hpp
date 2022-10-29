// MATLABPublisherInterface.hpp
// Copyright 2019-2021 The MathWorks, Inc.

#ifndef MATLABSVCCLIENTINTEFACE_H
#define MATLABSVCCLIENTINTEFACE_H

#include "MATLABInterfaceCommon.hpp"
#include <mutex>
class MATLABROSMsgInterfaceBase;
class MATLABSvcClientInterface {

  protected:
    MDFactory_T mFactory;
    std::shared_ptr<ros::ServiceClient> mClient;
    int64_t mNextRequestId;
    bool mNextRequestTimeoutState;
    std::mutex mRequestTimeoutStateMutex;

    void setNextRequestTimeoutState(bool state) {
        // Synchronized mNextRequestTimeoutState as it will be used by
        // 2 different threads and to make sure that while one is reading
        // or writing another should wait for the latest update.
        std::unique_lock<std::mutex> lck(mRequestTimeoutStateMutex);
        mNextRequestTimeoutState = state;
    }
    bool getNextRequestTimeoutState() {
        // Synchronized mNextRequestTimeoutState as it will be used by
        // 2 different threads and to make sure that while one is reading
        // or writing another should wait for the latest update.
        std::unique_lock<std::mutex> lck(mRequestTimeoutStateMutex);
        return mNextRequestTimeoutState;
    }

  public:
    MultiLibLoader mMultiLibLoader;
    std::map<std::string,boost::shared_ptr<MATLABROSMsgInterfaceBase>>* mCommonObjMap; 
    explicit MATLABSvcClientInterface() {
    }

    virtual ~MATLABSvcClientInterface() {
    }

    virtual intptr_t createSvcClient(const std::string& /* Svc_name */,
                                     std::shared_ptr<ros::NodeHandle> /* node */) {
        return 0;
    }

    virtual bool send_request(void* /* sd */,
                              SendDataToMATLABFunc_T /* sendDataToMATLABFunc */,
                              const intptr_t /* hSvcClient */) {
        return false;
    }

    virtual void appendAndSendToMATLAB(void* sd,
                                       SendDataToMATLABFunc_T sendDataToMATLABFunc,
                                       MDArray_T arr,
                                       const intptr_t hSvcClient,
                                       const int64_t requestId = 0) {

        auto hndlArr = mFactory.createStructArray({1, 1}, {"handle"});
        hndlArr[0]["handle"] = mFactory.createScalar(CONVERT_INTPTR_T_FOR_MATLAB_ARRAY(hSvcClient));

        auto requestIdArr = mFactory.createStructArray({1, 1}, {"requestId"});
        requestIdArr[0]["requestId"] = mFactory.createScalar(requestId);

        std::vector<MDArray_T> data{arr, hndlArr, requestIdArr};
        sendDataToMATLABFunc(sd, data);
    }

    virtual bool setNextRequest() {
        return false;
    }

    virtual void setCommonObjMap(std::map<std::string,boost::shared_ptr<MATLABROSMsgInterfaceBase>>* ){}

#ifndef FOUNDATION_MATLABDATA_API
    virtual bool addRequestToPool(matlab::data::StructArray /*arr*/)     
#else
    virtual bool addRequestToPool(foundation::matlabdata::StructArray /*arr*/)     
#endif
    {
        return false;
    }
    
    virtual bool clientRequestTimeout(int64_t /*requestId*/) {
        return false;
    }
    virtual intptr_t getCurrentReqId() const {
        return 0;
    }
#ifndef FOUNDATION_MATLABDATA_API
    int64_t get_requestId_from_arr(const matlab::data::StructArray& arr) {
        // Get the request ID
        int64_t requestId = 0;
        try {
            // data
            const matlab::data::TypedArray<int64_t> data_arr = arr[0]["requestId"];
            requestId = data_arr[0];
        } catch (matlab::data::InvalidFieldNameException&) {
            throw std::invalid_argument("Field 'requestId' is missing.");
        } catch (matlab::data::TypeMismatchException&) {
            throw std::invalid_argument("Field 'requestId' is wrong type; expected a int64.");
        }
        return requestId;
    }
#endif
};


#endif // MATLABSVCCLIENTINTEFACE_H
