/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/gazeboclient/GetModelInfoAction.hpp"

namespace robotics {

namespace gazebotransport {

GetModelInfoAction::GetModelInfoAction()
    : m_message()
    , m_buffer(nullptr) {
    // setup "get Gazebo model info" message
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_GET_MODEL_INFO);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_get_model_info()->set_topic_name("");
}

std::string GetModelInfoAction::getMsgToSend() {
    return m_message.SerializeAsString();
}

void GetModelInfoAction::processMsg(boost::optional<std::string> const& msg) {
    m_buffer = nullptr;
    if (msg) {
        auto reply = std::make_shared<mw::internal::robotics::gazebotransport::Packet>();
        if (reply->ParseFromString(*msg) && reply->has_model_info()) {
            m_buffer = reply;
        }
    }
}

std::shared_ptr<mw::internal::robotics::gazebotransport::Packet>
GetModelInfoAction::getModelInfo() {
    return m_buffer;
}
} // namespace gazebotransport
} // namespace robotics
