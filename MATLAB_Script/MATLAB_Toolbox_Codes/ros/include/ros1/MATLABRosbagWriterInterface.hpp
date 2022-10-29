// MATLABRosbagWriterInterface.hpp
// Copyright 2021 The MathWorks, Inc.

#ifndef MATLABROSBAGWRITERINTERFACE1_H
#define MATLABROSBAGWRITERINTERFACE1_H

#include <rosbag/bag.h>
using namespace rosbag;
#define Rosbag Bag*                                                                                \

#include "MATLABInterfaceCommon.hpp"
class MATLABROSMsgInterfaceBase;
class MATLABRosbagWriterInterface {

  public:
    MultiLibLoader mMultiLibLoader;
    std::map<std::string,boost::shared_ptr<MATLABROSMsgInterfaceBase>>* mCommonObjMap; 
    Rosbag mBag;
    explicit MATLABRosbagWriterInterface() {
    }
    
    virtual ~MATLABRosbagWriterInterface() {
    }

    virtual void setCommonObjMap(std::map<std::string,boost::shared_ptr<MATLABROSMsgInterfaceBase>>* ){}

#ifndef FOUNDATION_MATLABDATA_API
    virtual bool write(const std::string& /*topic*/, const uint32_t& /*sec*/, 
            const uint32_t& /*nSec*/, const matlab::data::StructArray& /* arr */)
#else
    virtual bool write(const std::string& /*topic*/, const uint32_t& /*sec*/, 
            const uint32_t& /*nSec*/, const foundation::matlabdata::StructArray& /* arr */)
#endif
    {
        return false;
    }
};


#endif // MATLABROSBAGWRITERINTERFACE1_H
