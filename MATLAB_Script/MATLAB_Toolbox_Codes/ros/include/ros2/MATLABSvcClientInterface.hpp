// MATLABPublisherInterface.hpp
// Copyright 2019-2021 The MathWorks, Inc.

#ifndef MATLABSVCCLIENTINTEFACE_H
#define MATLABSVCCLIENTINTEFACE_H

#ifndef FOUNDATION_MATLABDATA_API
#include "MDArray.hpp"
#include "StructArray.hpp"
#include "TypedArrayRef.hpp"
#include "Struct.hpp"
#include "ArrayFactory.hpp"
#include "StructRef.hpp"
#include "Reference.hpp"
#endif

#include "class_loader/multi_library_class_loader.hpp"
using namespace class_loader;
#define MultiLibLoader MultiLibraryClassLoader*

#ifndef DLL_IMPORT_SYM
#ifdef _MSC_VER
#define DLL_IMPORT_SYM __declspec(dllimport)
#else
#define DLL_IMPORT_SYM __attribute__((visibility("default")))
#endif
#endif

#ifndef FOUNDATION_MATLABDATA_API
typedef matlab::data::Array MDArray_T;
typedef matlab::data::ArrayFactory MDFactory_T;
#else
typedef foundation::matlabdata::Array MDArray_T;
typedef foundation::matlabdata::standalone::ClientArrayFactory MDFactory_T;
#endif

#if defined(__APPLE__)
// Clang has problem with intptr_t
#define CONVERT_INTPTR_T_FOR_MATLAB_ARRAY(val) (static_cast<int64_t>(val))
#define CONVERT_SIZE_T_FOR_MATLAB_ARRAY(val) (static_cast<uint64_t>(val))
#else
#define CONVERT_INTPTR_T_FOR_MATLAB_ARRAY(val) (val)
#define CONVERT_SIZE_T_FOR_MATLAB_ARRAY(val) (val)
#endif // defined(__APPLE__)
#include <mutex>
typedef bool (*SendDataToMATLABFunc_T)(void* sd, const std::vector<MDArray_T>& outData);
class MATLABROS2MsgInterfaceBase;
class MATLABSvcClientInterface {

  protected:
    MDFactory_T mFactory;
    int64_t mNextRequestId;
    bool mNextRequestTimeoutState;
    std::mutex mMutex;
    std::condition_variable mlResponseRecvCond; 
    std::condition_variable serverAvailableCond;
    volatile std::atomic<bool> mIfWaitingForResponse;
    std::shared_ptr<std::thread> mMoniterServerStatusThread;
    volatile std::atomic<bool> mIsActive;
    void* mSd;
    SendDataToMATLABFunc_T mSendDataToMATLABFunc;
    intptr_t mHsvcClient;
    void setNextRequestTimeoutState(bool state) {
        // Synchronized mNextRequestTimeoutState as it will be used by
        // 2 different threads and to make sure that while one is reading
        // or writing another should wait for the latest update.
        std::unique_lock<std::mutex> lck(mMutex);
        mNextRequestTimeoutState = state;
    }
    bool getNextRequestTimeoutState() {
        // Synchronized mNextRequestTimeoutState as it will be used by
        // 2 different threads and to make sure that while one is reading
        // or writing another should wait for the latest update.
        std::unique_lock<std::mutex> lck(mMutex);
        return mNextRequestTimeoutState;
    }
  public:
    RCLCPP_SMART_PTR_DEFINITIONS(MATLABSvcClientInterface)
    MultiLibLoader mMultiLibLoader;
    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* mCommonObjMap;
    explicit MATLABSvcClientInterface()
        : mIfWaitingForResponse(false)
        , mIsActive(true)
        , mSd(NULL)
        , mHsvcClient(0){
            // start the thread
            mMoniterServerStatusThread = std::make_shared<std::thread>(
                    &MATLABSvcClientInterface::monitorServerStatusAfterSendingRequest, this);
    }

    virtual ~MATLABSvcClientInterface() {
        mIsActive = false;
        
        // While deleting, notify the worker thread to release. 
        setWaitingForResponse(false);
        releaseTheWaitForResponse();
        
        {
            std::unique_lock<std::mutex> lck(mMutex);
            serverAvailableCond.notify_all();
        }
        
        if(mMoniterServerStatusThread.get()){
            mMoniterServerStatusThread->join();
        }
        
        mMoniterServerStatusThread.reset();
    }

    virtual intptr_t createSvcClient(const std::string& /* Svc_name */,
                                     const rclcpp::QoS& /* qos */,
                                     rclcpp::Node::SharedPtr /* node */) {
        return 0;
    }
    
    virtual void setCommonObjMap(std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* ){}
    
#ifndef FOUNDATION_MATLABDATA_API
    virtual bool send_request(void* /* sd */,
                              SendDataToMATLABFunc_T /* sendDataToMATLABFunc */,
                              const intptr_t /* hSvcClient */)
#else
    virtual bool send_request(void* /* sd */,
                              SendDataToMATLABFunc_T /* sendDataToMATLABFunc */,
                              const intptr_t /* hSvcClient */)
#endif
    {
        return false;
    }

    virtual void appendAndSendToMATLAB(void* sd,
                                       SendDataToMATLABFunc_T sendDataToMATLABFunc,
                                       MDArray_T arr,
                                       const intptr_t hSvcClient,
                                       const int64_t requestId) {

        auto hndlArr = mFactory.createStructArray({1, 1}, {"handle"});
        hndlArr[0]["handle"] = mFactory.createScalar(CONVERT_INTPTR_T_FOR_MATLAB_ARRAY(hSvcClient));
        
        auto requestIdArr = mFactory.createStructArray({1, 1}, {"requestId"});
        requestIdArr[0]["requestId"] = mFactory.createScalar(requestId);

        std::vector<MDArray_T> data{arr, hndlArr, requestIdArr};
        sendDataToMATLABFunc(sd, data);
    }
    
    virtual int64_t getCurrentReqId() const {
        return mNextRequestId;
    }
    
    virtual bool releaseTheWaitForResponse() {
        std::unique_lock<std::mutex> lck(mMutex);
        mlResponseRecvCond.notify_all();
        
        return true;
    }
    
    virtual bool waitForResponse() {
        std::unique_lock<std::mutex> lck(mMutex);
        serverAvailableCond.notify_all();
        mlResponseRecvCond.wait(lck);
        
        return true;
    }
    
    virtual void sendErrorMsgToMATLAB(const std::string& errName, 
                                            const std::string& errDetails) {
        auto outArray = mFactory.createStructArray({1, 1}, {"errName", "errDetails"});
        outArray[0]["errName"] = mFactory.createCharArray(errName);
        outArray[0]["errDetails"] = mFactory.createCharArray(errDetails);
        if (mSd != NULL && !getNextRequestTimeoutState()) {
            appendAndSendToMATLAB(mSd, mSendDataToMATLABFunc, outArray, mHsvcClient, mNextRequestId);
        }
    }
   
    virtual bool isWaitingForResponse(){
        std::unique_lock<std::mutex> lck(mMutex);
        return mIfWaitingForResponse; 
    }
    
    virtual void setWaitingForResponse(bool status){
        std::unique_lock<std::mutex> lck(mMutex);
        mIfWaitingForResponse = status; 
    }
    
    virtual void monitorServerStatusAfterSendingRequest() {
        // If there is any crash in service-server, ROS2 does not provide 
        // any mechanism to notify the service-client about the crash. So
        // client will be keep waiting for server to get the response.
        // In this case, to avoid the dead-lock, after request is sent,
        // server is being monitored until we get the response back using 
        // this function. If server crashes or anything goes wrong, this 
        // informs the worker thread not to wait for the response.
        // As we are checking periodically in a certain interval, this 
        // function is running in a different thread to avoid the delayed
        // response and it is controlled(start/stop) using
        // serverAvailableCond condition_variable.
        while (mIsActive) {
            {
                if(isWaitingForResponse()){
                    if(!ifServerAvailable(1)) {
                        sendErrorMsgToMATLAB("serviceServerNotAvailableError",
                                "service-server not available. Failed to communicate to service-server.");
                        setWaitingForResponse(false);
                        releaseTheWaitForResponse();
                    }
                }else{
                    std::unique_lock<std::mutex> lck(mMutex);
                    serverAvailableCond.wait(lck);
                }
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    
    virtual bool setNextRequest() {
        return false;
    }

#ifndef FOUNDATION_MATLABDATA_API
    virtual int64_t addRequestToPool(matlab::data::StructArray /*arr*/)
#else
    virtual int64_t addRequestToPool(foundation::matlabdata::StructArray /*arr*/)
#endif    
    {
        return 0;
    }
    
    virtual bool clientRequestTimeout(int64_t /*requestId*/) {
        return false;
    }
    
    virtual bool ifServerAvailable(int64_t /*timeoutMs*/) {
        return false;
    }
};


#endif // MATLABSVCCLIENTINTEFACE_H
