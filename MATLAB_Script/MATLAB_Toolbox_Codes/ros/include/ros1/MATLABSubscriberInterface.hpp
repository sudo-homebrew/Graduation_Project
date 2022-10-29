// MATLABSubscriberInterface.hpp
// Copyright 2019-2021 The MathWorks, Inc.

#ifndef MATLABSUBSCRIBERINTERFACE1_H
#define MATLABSUBSCRIBERINTERFACE1_H

#include "MATLABInterfaceCommon.hpp"
class MATLABROSMsgInterfaceBase;
class MATLABSubscriberInterface {
  protected:
    MDFactory_T mFactory;

  public:
    MultiLibLoader mMultiLibLoader;
    std::map<std::string,boost::shared_ptr<MATLABROSMsgInterfaceBase>>* mCommonObjMap; 
    explicit MATLABSubscriberInterface() {
    }

    virtual ~MATLABSubscriberInterface() {
    }

    virtual intptr_t createSubscription(const std::string& /* topic_name */,
                                        std::shared_ptr<ros::NodeHandle> /* node */,
                                        void* /* sd */,
                                        SendDataToMATLABFunc_T, /* sendDataToMATLABFunc */
                                        uint32_t) {
        return 0;
    }

    virtual void setCommonObjMap(std::map<std::string,boost::shared_ptr<MATLABROSMsgInterfaceBase>>* ){}

    virtual void appendAndSendToMATLAB(void* sd,
                                       SendDataToMATLABFunc_T sendDataToMATLABFunc,
                                       MDArray_T& arr) {
        std::vector<MDArray_T> data{arr};
        sendDataToMATLABFunc(sd, data);
    }
};

#endif // MATLABSUBSCRIBERINTERFACE1_H
