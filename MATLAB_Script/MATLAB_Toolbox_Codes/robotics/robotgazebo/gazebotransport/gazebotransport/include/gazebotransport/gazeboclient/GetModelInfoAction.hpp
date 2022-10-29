/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBO_GET_MODEL_INFO_ACTION_HPP
#define GAZEBO_GET_MODEL_INFO_ACTION_HPP

#include "gazebotransport/gazeboclient/Action.hpp"
#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h"

#include <cstdint> // for uint32_t
#include <string>

namespace robotics {
namespace gazebotransport {
/// GetModelInfoAction query Gazebo joint and link names
/**
This action sends a GetModelInfo request and expect a ModelInfo packet reply
A Model in Gazebo consists multiple joints and links
This action gets information from all the models in Gazebo
*/
class GetModelInfoAction : public Action {
  public:
    /// constructor
    GetModelInfoAction();

    /// create the serialized GetModelInfo message
    /**
    @return                      serialized GetModelInfo message
    */
    std::string getMsgToSend() override;

    /// process the serialized server reply
    /**
    @param msg                   optional serialized server reply

    If msg is boost::none or cannot be deserialized into ModelInfo Packet,
    This action is not successful
    */
    void processMsg(boost::optional<std::string> const& msg) override;

    /// return the model info obtained from Gazebo
    std::shared_ptr<mw::internal::robotics::gazebotransport::Packet> getModelInfo();

  private:
    /// Packet contains GetModelInfo message
    mw::internal::robotics::gazebotransport::Packet m_message;

    /// Contains the last received ModelInfo packet
    std::shared_ptr<mw::internal::robotics::gazebotransport::Packet> m_buffer;
};
} // namespace gazebotransport
} // namespace robotics

#endif
