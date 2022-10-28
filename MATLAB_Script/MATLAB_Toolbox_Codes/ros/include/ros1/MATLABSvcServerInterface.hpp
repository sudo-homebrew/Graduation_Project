// MATLABSubscriberInterface.hpp
// Copyright 2019-2021 The MathWorks, Inc.

#ifndef MATLABSVCSERVERINTERFACE_H
#define MATLABSVCSERVERINTERFACE_H

#include "MATLABInterfaceCommon.hpp"
class MATLABROSMsgInterfaceBase;
class MATLABSvcServerInterface {
  protected:
    MDFactory_T mFactory;
    std::shared_ptr<ros::ServiceServer> mSvc;
    bool mCallbackError;

  public:
    MultiLibLoader mMultiLibLoader;
    std::map<std::string,boost::shared_ptr<MATLABROSMsgInterfaceBase>>* mCommonObjMap; 
    explicit MATLABSvcServerInterface() {
    }

    virtual ~MATLABSvcServerInterface() {
    }

    virtual intptr_t createSvcServer(const std::string& /* Svc_name */,
                                     std::shared_ptr<ros::NodeHandle> /* node */,
                                     void* /* sd */,
                                     SendDataToMATLABFunc_T /* sendDataToMATLABFunc */,
                                     const intptr_t /* hSvcServer */) {
        return 0;
    }

    virtual void appendAndSendToMATLAB(void* sd,
                                       SendDataToMATLABFunc_T sendDataToMATLABFunc,
                                       MDArray_T arr,
                                       const intptr_t hSvcServer) {
        auto hndlArr = mFactory.createStructArray({1, 1}, {"handle"});
        hndlArr[0]["handle"] = mFactory.createScalar(CONVERT_INTPTR_T_FOR_MATLAB_ARRAY(hSvcServer));
        std::vector<MDArray_T> data{arr, hndlArr};
        sendDataToMATLABFunc(sd, data);
    }

    virtual void setCommonObjMap(std::map<std::string,boost::shared_ptr<MATLABROSMsgInterfaceBase>>* ){}

#ifndef FOUNDATION_MATLABDATA_API
    virtual bool setResponseFromMatlab(matlab::data::StructArray /*arr*/)
#else
    virtual bool setResponseFromMatlab(foundation::matlabdata::StructArray /*arr*/)
#endif
    {
        return false;
    }    

    virtual void setCallbackError(bool callbackError) {
        mCallbackError = callbackError;
    }
    virtual bool getCallbackError() {
        return mCallbackError;
    }
};
#endif // MATLABSVCSERVERINTERFACE1_H
