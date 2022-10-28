/* Copyright 2019 The MathWorks, Inc. */

#include "gazebotransport/gazeboclient/InitSubscribeCustomMessageAction.hpp"
#include "gazebotransport/gazeboclient/checkErrorStatus.hpp"
#include "iostream"
namespace robotics {
namespace gazebotransport {
InitSubscribeCustomMessageAction::InitSubscribeCustomMessageAction()
    : m_message()
    , m_success(false) {
    // setup default init subscribe custom message
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_INIT_CUSTOM_MESSAGE_SUBSCRIBER);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_init_custom_subscriber()->set_topic_name("");
    m_message.mutable_init_custom_subscriber()->set_message_type("");
}

std::string InitSubscribeCustomMessageAction::getMsgToSend() {
    return m_message.SerializeAsString();
}

void InitSubscribeCustomMessageAction::processMsg(boost::optional<std::string> const& msg) {
    m_success = false;

    auto status = checkErrorStatus(msg);

    if (status &&
        *status ==
            mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE) {
        m_success = true;
    }
}

void InitSubscribeCustomMessageAction::setCustomMsg(std::string const& topic_name,
                                                    std::string const& message_type) {
    m_message.mutable_init_custom_subscriber()->set_topic_name(topic_name);
    m_message.mutable_init_custom_subscriber()->set_message_type(message_type);
}

bool InitSubscribeCustomMessageAction::success() const {
    return m_success;
}
} // namespace gazebotransport
} // namespace robotics
