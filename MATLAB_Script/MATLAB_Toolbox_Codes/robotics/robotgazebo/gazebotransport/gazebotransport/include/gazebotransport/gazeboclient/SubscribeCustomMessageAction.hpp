/* Copyright 2019 The MathWorks, Inc. */
#ifndef SUBSCRIBE_CUSTOM_MESSAGE_ACTION_HPP
#define SUBSCRIBE_CUSTOM_MESSAGE_ACTION_HPP

#include "gazebotransport/gazeboclient/Action.hpp"
#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h"
#include <cstdint> // for uint32_t
#include <string>

namespace robotics {
namespace gazebotransport {
/// SubscribeCustomMessageAction reads custom message from Gazebo server
class SubscribeCustomMessageAction : public Action {
  public:
    SubscribeCustomMessageAction();

    /// setup subscribe command
    /**
    @param topic_name        Topic name of the custom message subscriber
    @param message_type      Message type of the custom message
    */
    void setCustomMsg(std::string const& topic_name, std::string const& message_type);

    /// create the serialized subscribe custom message
    /**
    @return                      serialized subscribe custom message command
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
    std::pair<bool, std::shared_ptr<mw::internal::robotics::gazebotransport::Packet>> getMessage();

    /// initialize empty packet buffer
    std::shared_ptr<mw::internal::robotics::gazebotransport::Packet> initalizeMessage(
        std::shared_ptr<mw::internal::robotics::gazebotransport::Packet>&& input);

  private:
    /// Packet contains Subscribed custom message
    mw::internal::robotics::gazebotransport::Packet m_message;

    std::shared_ptr<mw::internal::robotics::gazebotransport::Packet> m_buffer;

    /// indicates whether the action is successful
    bool m_isNew;
};
} // namespace gazebotransport
} // namespace robotics

#endif
