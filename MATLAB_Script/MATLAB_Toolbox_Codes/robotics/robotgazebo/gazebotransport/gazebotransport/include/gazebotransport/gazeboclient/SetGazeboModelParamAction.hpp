/* Copyright 2020 The MathWorks, Inc. */
#ifndef SET_GAZEBO_MODEL_PARAM_ACTION_HPP
#define SET_GAZEBO_MODEL_PARAM_ACTION_HPP

#include "gazebotransport/gazeboclient/Action.hpp"
#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h" // protobuf auto-generated header

#include <cstdint> // for uint32_t
#include <string>

namespace robotics {
namespace gazebotransport {
/// SetGazeboModelParamAction sends an Set Gazebo model param command to Gazebo server
class SetGazeboModelParamAction : public Action {
  public:
    SetGazeboModelParamAction();

    /// setup the set gazebo model param command
    /**
    @param message                Gazebo model protobuf message
    */
    void setGazeboModelParam(mw::internal::robotics::gazebotransport::Gazebomodel& message);

    /// create the serialized Gazebo model message
    /**
    @return                      serialized Gazebo model message
    */
    std::string getMsgToSend() override;

    /// process the serialized server reply
    /**
    @param msg                   optional serialized server reply

    If msg is boost::none or cannot be deserialized into Packet with CoSimError::None status
    This action is not successful
    */
    void processMsg(boost::optional<std::string> const& msg) override;

    /// check whether last action is successful
    std::pair<uint8_t, std::string> success() const;

  private:
    /// m_message Buffered Gazebo model command
    mw::internal::robotics::gazebotransport::Packet m_message;

    /// m_success Record whether last set action is successful
    uint8_t m_success;

    std::string m_error_message;
};
} // namespace gazebotransport
} // namespace robotics

#endif
