/* Copyright 2019 The MathWorks, Inc. */

#include "gazebotransport/gazeboclient/SubscribeCustomMessageAction.hpp"
#include "gazebotransport/gazeboclient/checkErrorStatus.hpp"
#include "iostream"
namespace robotics {
namespace gazebotransport {
SubscribeCustomMessageAction::SubscribeCustomMessageAction()
    : m_message()
    , m_buffer(nullptr)
    , m_isNew(false) {
    // setup default subscribe custom message
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_CUSTOM_MESSAGE_SUBSCRIBER);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_request_custom_message_support()->set_topic_name("");
    m_message.mutable_request_custom_message_support()->set_message_type("");
}

std::string SubscribeCustomMessageAction::getMsgToSend() {
    return m_message.SerializeAsString();
}

std::shared_ptr<mw::internal::robotics::gazebotransport::Packet>
SubscribeCustomMessageAction::initalizeMessage(
    std::shared_ptr<mw::internal::robotics::gazebotransport::Packet>&& input) {
    input->mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                        PacketHeader_MsgID_CUSTOM_MESSAGE_SUBSCRIBER);
    input->mutable_header()->mutable_time_stamp()->set_seconds(0);
    input->mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    input->mutable_custom_message_support()->set_topic_name("");
    input->mutable_custom_message_support()->set_message_type("");
    input->mutable_custom_message_support()->set_data("");
    input->mutable_custom_message_support()->set_is_new(false);
    return std::move(input);
}

void SubscribeCustomMessageAction::processMsg(boost::optional<std::string> const& msg) {

    if (msg) {
        auto reply = std::make_shared<mw::internal::robotics::gazebotransport::Packet>();
        if (reply->ParseFromString(*msg) && reply->has_custom_message_support()) {
            // if serialized data string (custom message) is empty
            // then returns false
            if (reply->custom_message_support().data() == "") {
                m_isNew = false;
                m_buffer = nullptr;
            } else {
                m_isNew = reply->custom_message_support().is_new();
                m_buffer = reply;
            }
        } else {
            m_buffer = nullptr;
            m_isNew = false;
        }
    } else {
        m_buffer = nullptr;
        m_isNew = false;
    }

    if (!m_buffer) {
        m_buffer = this->initalizeMessage(
            std::make_shared<mw::internal::robotics::gazebotransport::Packet>());
    }
}

void SubscribeCustomMessageAction::setCustomMsg(std::string const& topic_name,
                                                std::string const& message_type) {
    m_message.mutable_request_custom_message_support()->set_topic_name(topic_name);
    m_message.mutable_request_custom_message_support()->set_message_type(message_type);
}

std::pair<bool, std::shared_ptr<mw::internal::robotics::gazebotransport::Packet>>
SubscribeCustomMessageAction::getMessage() {
    return std::make_pair(m_isNew, m_buffer);
}
} // namespace gazebotransport
} // namespace robotics
