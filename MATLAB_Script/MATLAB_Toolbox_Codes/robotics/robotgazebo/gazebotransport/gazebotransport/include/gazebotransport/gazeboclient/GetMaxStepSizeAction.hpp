/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBO_GET_MAX_STEP_SIZE_ACTION_HPP
#define GAZEBO_GET_MAX_STEP_SIZE_ACTION_HPP

#include "gazebotransport/gazeboclient/Action.hpp"
#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h"

#include <cstdint> // for uint32_t
#include <string>

namespace robotics {
namespace gazebotransport {
/// GetMaxStepSizeAction query Gazebo physics solver max step size
/**
This action sends a MaxStepSize get request and expect a reply with max step size
*/
class GetMaxStepSizeAction : public Action {
  public:
    /// constructor
    GetMaxStepSizeAction();

    /// create the serialized MaxStepSize message
    /**
    @return                      serialized MaxStepSize message
    */
    std::string getMsgToSend() override;

    /// process the serialized server reply
    /**
    @param msg                   optional serialized server reply

    If msg is boost::none or cannot be deserialized into TopicList Packet,
    This action is not successful
    */
    void processMsg(boost::optional<std::string> const& msg) override;

    /// return the max step size obtained from Gazebo physics solver
    double getMaxStepSize();

  private:
    /// Packet contains GetMaxStepSizeAction message
    mw::internal::robotics::gazebotransport::Packet m_message;

    /// Contains the last received max step size
    double m_maxStepSize;
};
} // namespace gazebotransport
} // namespace robotics

#endif
