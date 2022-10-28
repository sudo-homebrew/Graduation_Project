// MATLABActClientInterface.hpp
// Copyright 2020-2021 The MathWorks, Inc.

#ifndef MATLABACTCLIENTINTERFACE_H
#define MATLABACTCLIENTINTERFACE_H

#include "MATLABInterfaceCommon.hpp"
class MATLABROSMsgInterfaceBase;
class MATLABActClientInterface {
  protected:
    MDFactory_T mFactory;

  public:
    std::map<std::string,boost::shared_ptr<MATLABROSMsgInterfaceBase>>* mCommonObjMap; 
    MultiLibLoader mMultiLibLoader;
    explicit MATLABActClientInterface() {
    }

    virtual ~MATLABActClientInterface() {
    }

    virtual intptr_t createActClient(const std::string& /* act_name */,
                                     std::shared_ptr<ros::NodeHandle> /* node */) {
        return 0;
    }
    
    virtual bool waitForActServer(intptr_t /*timeout*/){
        return false;
    }

    virtual void setCommonObjMap(std::map<std::string,boost::shared_ptr<MATLABROSMsgInterfaceBase>>* ){}

#ifndef FOUNDATION_MATLABDATA_API
    virtual bool sendGoaltoActserver(const matlab::data::StructArray /* arr */,
                                      void* /* sd */,
                                      SendDataToMATLABFunc_T /* onGoalActiveCBFunc */,
                                      SendDataToMATLABFunc_T /* onFeedbackReceivedCBFunc */,
                                      SendDataToMATLABFunc_T /* onResultReceivedCBFunc */,
                                      const intptr_t /* hActClient */)
#else
    virtual bool sendGoaltoActserver(const foundation::matlabdata::StructArray /* arr */,
                                      void* /* sd */,
                                      SendDataToMATLABFunc_T /* onGoalActiveCBFunc */,
                                      SendDataToMATLABFunc_T /* onFeedbackReceivedCBFunc */,
                                      SendDataToMATLABFunc_T /* onResultReceivedCBFunc */,
                                      const intptr_t /* hActClient */)
#endif
    {
        return false;
    }

    virtual bool cancelGoal(){
        return false;
    }
    
    virtual bool cancelAllGoal(){
        return false;
    }
    
    virtual bool isServerConnected(){
        return false;
    }
    
    virtual int32_t getState(){
        return -1;
    }
    
    virtual MDArray_T getResult(){
        MDArray_T res;
        return res;
    }
    
    virtual void appendAndSendToMATLAB(void* sd,
                                       SendDataToMATLABFunc_T sendDataToMATLABFunc,
                                       MDArray_T arr,
                                       const intptr_t hActServer) {
        auto hndlArr = mFactory.createStructArray({1, 1}, {"handle"});
        hndlArr[0]["handle"] = mFactory.createScalar(CONVERT_INTPTR_T_FOR_MATLAB_ARRAY(hActServer));
        std::vector<MDArray_T> data{arr, hndlArr};
        sendDataToMATLABFunc(sd, data);
    }
    
    virtual void appendAndSendToMATLAB(void* sd,
                                       SendDataToMATLABFunc_T sendDataToMATLABFunc,
                                       MDArray_T arr,
                                       const intptr_t hActServer,
                                       int32_t goal_state) {
        auto hndlArr = mFactory.createStructArray({1, 1}, {"handle"});
        hndlArr[0]["handle"] = mFactory.createScalar(CONVERT_INTPTR_T_FOR_MATLAB_ARRAY(hActServer));
        
        auto goalStateArr = mFactory.createStructArray({1, 1}, {"goal_state"});
        goalStateArr[0]["goal_state"] = mFactory.createScalar(goal_state);
        std::vector<MDArray_T> data{arr, hndlArr, goalStateArr};
        sendDataToMATLABFunc(sd, data);
    }    
};
#endif // MATLABACTCLIENTINTERFACE_H
