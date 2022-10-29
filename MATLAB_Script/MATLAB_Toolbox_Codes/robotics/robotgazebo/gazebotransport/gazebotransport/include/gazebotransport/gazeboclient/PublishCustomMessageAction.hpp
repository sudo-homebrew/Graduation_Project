/* Copyright 2019 The MathWorks, Inc. */
#ifndef PUBLISH_CUSTOM_MESSAGE_ACTION_HPP
#define PUBLISH_CUSTOM_MESSAGE_ACTION_HPP

#include "gazebotransport/gazeboclient/Action.hpp"
#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h"
#include <cstdint> // for uint32_t
#include <string>

namespace robotics {
namespace gazebotransport {
/// PublishCustomMessageAction sends custom message to Gazebo server
class PublishCustomMessageAction : public Action {
  public:
    PublishCustomMessageAction();

    /// setup the torque command
    /**
    @param topic_name                Topic name of the custom message publisher
    @param message_type              Message type of the custom message
    @param data_string               Serialized custom message string
    */
    void setCustomMsg(std::string const& topic_name,
                      std::string const& message_type,
                      std::string const& data_string);

    /// create the serialized custom message
    /**
    @return                      serialized custom message
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
    bool success() const;

  private:
    // Co-Sim Packet with custom message
    mw::internal::robotics::gazebotransport::Packet m_message;

    /// m_success Record whether last apply action is successful
    bool m_success;
};
} // namespace gazebotransport
} // namespace robotics

#endif
