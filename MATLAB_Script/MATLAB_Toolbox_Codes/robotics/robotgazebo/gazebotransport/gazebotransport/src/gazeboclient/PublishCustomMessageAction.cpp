/* Copyright 2019 The MathWorks, Inc. */

#include "gazebotransport/gazeboclient/PublishCustomMessageAction.hpp"
#include "gazebotransport/gazeboclient/checkErrorStatus.hpp"
#include "iostream"
namespace robotics {
namespace gazebotransport {
PublishCustomMessageAction::PublishCustomMessageAction()
    : m_message()
    , m_success(false) {
    // setup default publish custom message
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_CUSTOM_MESSAGE_PUBLISHER);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_custom_message_support()->set_topic_name("");
    m_message.mutable_custom_message_support()->set_message_type("");
    m_message.mutable_custom_message_support()->set_data("");
}

std::string PublishCustomMessageAction::getMsgToSend() {
    return m_message.SerializeAsString();
}

void PublishCustomMessageAction::processMsg(boost::optional<std::string> const& msg) {
    m_success = false;

    auto status = checkErrorStatus(msg);

    if (status &&
        *status ==
            mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE) {
        m_success = true;
    }
}

void PublishCustomMessageAction::setCustomMsg(std::string const& topic_name,
                                              std::string const& message_type,
                                              std::string const& data_string) {
    m_message.mutable_custom_message_support()->set_topic_name(topic_name);
    m_message.mutable_custom_message_support()->set_message_type(message_type);
    m_message.mutable_custom_message_support()->set_data(data_string);
}

bool PublishCustomMessageAction::success() const {
    return m_success;
}
} // namespace gazebotransport
} // namespace robotics