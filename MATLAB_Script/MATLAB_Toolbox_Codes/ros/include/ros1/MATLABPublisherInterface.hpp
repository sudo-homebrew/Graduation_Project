// MATLABPublisherInterface.hpp
// Copyright 2019-2021 The MathWorks, Inc.

#ifndef MATLABPUBLISHERINTERFACE1_H
#define MATLABPUBLISHERINTERFACE1_H

#include "MATLABInterfaceCommon.hpp"
class MATLABROSMsgInterfaceBase;
class MATLABPublisherInterface {

  public:
    MultiLibLoader mMultiLibLoader;
    std::map<std::string,boost::shared_ptr<MATLABROSMsgInterfaceBase>>* mCommonObjMap; 

    explicit MATLABPublisherInterface() {
    }
    
    virtual ~MATLABPublisherInterface() {
    }

    virtual intptr_t createPublisher(const std::string& /* topic_name */,
                                     bool, /*latching */
                                     std::shared_ptr<ros::NodeHandle> /* node */) {
        return 0;
    }

    virtual void setCommonObjMap(std::map<std::string,boost::shared_ptr<MATLABROSMsgInterfaceBase>>* ){}

#ifndef FOUNDATION_MATLABDATA_API
    virtual bool publish(const matlab::data::StructArray& /* arr */)
#else
    virtual bool publish(const foundation::matlabdata::StructArray& /* arr */)
#endif
    {
        return false;
    }
};


#endif // MATLABPUBLISHERINTERFACE1_H
