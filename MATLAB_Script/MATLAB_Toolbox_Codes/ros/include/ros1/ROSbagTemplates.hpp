// ROSbagTemplates.hpp
// Copyright 2021 The MathWorks, Inc.

#ifndef ROSBAGTEMPLATES_H
#define ROSBAGTEMPLATES_H

#include "MATLABRosbagWriterInterface.hpp"
#include "MATLABROSMsgInterface.hpp"

template<class RosMessageType, class CommonType>
class ROSBagWriterImpl : public MATLABRosbagWriterInterface {
  CommonType mCommonObj;
  public:
            
    ROSBagWriterImpl()
        : MATLABRosbagWriterInterface(){
    }
    virtual void setCommonObjMap(std::map<std::string,boost::shared_ptr<MATLABROSMsgInterfaceBase>>* commonObjMap){
        mCommonObjMap = commonObjMap;
        mCommonObj.mCommonObjMap = mCommonObjMap;
    }
    virtual ~ROSBagWriterImpl() {
    }
    
    /*
        Writes the message into the bag file with topic and time-stamp
    */
    virtual bool write(const std::string& topic, const uint32_t& sec, 
            const uint32_t& nSec ,const matlab::data::StructArray& arr) {
        auto msg = boost::make_shared<RosMessageType>();
        mCommonObj.copy_from_struct(msg.get(), arr[0], mMultiLibLoader);
        mBag->write(topic, ros::Time(sec, nSec), msg);
        return true;
    }
};
#endif // ROSBAGTEMPLATES_H