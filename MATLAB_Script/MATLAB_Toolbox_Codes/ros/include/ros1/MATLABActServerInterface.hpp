// MATLABActServerInterface.hpp
// Copyright 2020-2021 The MathWorks, Inc.

#ifndef MATLABActServerInterface_H
#define MATLABActServerInterface_H

#include "MATLABInterfaceCommon.hpp"
class MATLABROSMsgInterfaceBase;
class MATLABActServerInterface {
  protected:
    MDFactory_T mFactory;

  public:
    MultiLibLoader mMultiLibLoader;
    std::map<std::string,boost::shared_ptr<MATLABROSMsgInterfaceBase>>* mCommonObjMap; 
    explicit MATLABActServerInterface() {
    }

    virtual ~MATLABActServerInterface() {
    }

    virtual intptr_t createActServer(const std::string& /* act_name */,
                                     std::shared_ptr<ros::NodeHandle> /* node */) {
        std::cout<<"\n base class !";
        return 0;
    }
    
    virtual void setCommonObjMap(std::map<std::string,boost::shared_ptr<MATLABROSMsgInterfaceBase>>* ){}

    virtual intptr_t addActServer(const std::string& /* act_name */,
                                  bool /*enableGoalExecCallback*/,
                                  std::shared_ptr<ros::NodeHandle> /* node */,
                                  void* /* sd */,
                                  SendDataToMATLABFunc_T /* newGoalCBFunc */,
                                  SendDataToMATLABFunc_T /* executeGoalCBFunc */,
                                  SendDataToMATLABFunc_T /* preemptCBFunc */,
                                  const intptr_t /* hSvcServer */) {
        std::cout<<"\n base class !";
        return 0;
    }
    
    virtual MDArray_T acceptNewGoal(){
        MDArray_T res;
        return res;
    }
    
    virtual bool isActive(){
        return false;
    }
    
    virtual bool isNewGoalAvailable(){
        return false;
    }
    
    virtual bool isPreemptRequested(){
        return false;
    }

#ifndef FOUNDATION_MATLABDATA_API
    virtual void publishFeedback(const matlab::data::StructArray /* arr */){}
#else
    virtual void publishFeedback(const foundation::matlabdata::StructArray /* arr */){}
#endif
    
#ifndef FOUNDATION_MATLABDATA_API
    virtual void setAborted(const matlab::data::StructArray /* arr */, 
                            std::string /* text */){}
#else
    virtual void setAborted(const foundation::matlabdata::StructArray /* arr */,
                            std::string /* text */){}
#endif

#ifndef FOUNDATION_MATLABDATA_API
    virtual void setPreempted(const matlab::data::StructArray /* arr */, 
                              std::string /* text */){}
#else
    virtual void setPreempted(const foundation::matlabdata::StructArray /* arr */,
                              std::string /* text */){}
#endif

#ifndef FOUNDATION_MATLABDATA_API
    virtual void setSucceeded(const matlab::data::StructArray /* arr */, 
                              std::string /* text */){}
#else
    virtual void setSucceeded(const foundation::matlabdata::StructArray /* arr */, 
                              std::string /* text */){}
#endif

    virtual void appendAndSendToMATLAB(void* sd,
                                       SendDataToMATLABFunc_T sendDataToMATLABFunc,
                                       MDArray_T arr,
                                       const intptr_t hActServer) {
        auto hndlArr = mFactory.createStructArray({1, 1}, {"handle"});
        hndlArr[0]["handle"] = mFactory.createScalar(CONVERT_INTPTR_T_FOR_MATLAB_ARRAY(hActServer));
        std::vector<MDArray_T> data{arr, hndlArr};
        sendDataToMATLABFunc(sd, data);
    }
};
#endif // MATLABActServerInterface_H
