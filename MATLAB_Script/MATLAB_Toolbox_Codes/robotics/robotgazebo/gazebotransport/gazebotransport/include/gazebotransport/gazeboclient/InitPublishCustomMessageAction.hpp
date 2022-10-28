/* Copyright 2019 The MathWorks, Inc. */
#ifndef INIT_PUBLISH_CUSTOM_MESSAGE_ACTION_HPP
#define INIT_PUBLISH_CUSTOM_MESSAGE_ACTION_HPP

#include "gazebotransport/gazeboclient/Action.hpp"
#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h"
#include <cstdint> // for uint32_t
#include <string>

namespace robotics {
namespace gazebotransport {
/// InitPublishCustomMessageAction initialize custom message publisher on Gazebo server
class InitPublishCustomMessageAction : public Action {
  public:
    InitPublishCustomMessageAction();

    /// setup custom message publisher
    /**
    @param topic_name               Topic name of the publisher
    @param message_type             Message type of custom message
    */
    void setCustomMsg(std::string const& topic_name, std::string const& message_type);

    /// create the serialized custom message init publisher packet
    /**
    @return                      serialized custom message packet
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
    /// Co-Sim Packet message
    mw::internal::robotics::gazebotransport::Packet m_message;

    /// m_success Record whether last apply action is successful
    bool m_success;
};
} // namespace gazebotransport
} // namespace robotics

#endif
