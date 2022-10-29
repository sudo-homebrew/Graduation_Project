/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/gazeboclient/GetTopicListAction.hpp"

namespace robotics {

namespace gazebotransport {

GetTopicListAction::GetTopicListAction()
    : m_message()
    , m_buffer(nullptr) {
    // setup get topic list message
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_GET_TOPIC_LIST);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_get_topic_list()->set_topic_name("");
}

std::string GetTopicListAction::getMsgToSend() {
    return m_message.SerializeAsString();
}

void GetTopicListAction::processMsg(boost::optional<std::string> const& msg) {
    m_buffer = nullptr;
    if (msg) {
        auto reply = std::make_shared<mw::internal::robotics::gazebotransport::Packet>();
        if (reply->ParseFromString(*msg) && reply->has_topic_list()) {
            m_buffer = reply;
        }
    }
}

std::shared_ptr<mw::internal::robotics::gazebotransport::Packet>
GetTopicListAction::getTopicList() {
    return m_buffer;
}
} // namespace gazebotransport
} // namespace robotics
