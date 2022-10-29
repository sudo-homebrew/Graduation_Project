// MATLABPublisherInterface.hpp
// Copyright 2017-2021 The MathWorks, Inc.

#ifndef MATLABPUBLISHERINTEFACE_H
#define MATLABPUBLISHERINTEFACE_H

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
class MATLABROS2MsgInterfaceBase;
class MATLABPublisherInterface {
    rmw_gid_t mNullGID;

  public:
    RCLCPP_SMART_PTR_DEFINITIONS(MATLABPublisherInterface)
    MultiLibLoader mMultiLibLoader;
    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* mCommonObjMap;
    explicit MATLABPublisherInterface()
        : mNullGID() {
    }

    virtual ~MATLABPublisherInterface() {
    }

    virtual intptr_t createPublisher(const std::string& /* topic_name */,
                                     const rclcpp::QoS& /* qos_profile */,
                                     rclcpp::Node::SharedPtr /* node */) {
        return 0;
    }
    
    virtual void setCommonObjMap(std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* ){}
    
#ifndef FOUNDATION_MATLABDATA_API
    virtual bool publish(const matlab::data::StructArray& /* arr */)
#else
    virtual bool publish(const foundation::matlabdata::StructArray& /* arr */)
#endif
    {
        return false;
    }

    virtual const rmw_gid_t& get_gid() const {
        return mNullGID;
    }
};


#endif // MATLABPUBLISHERINTEFACE_H
