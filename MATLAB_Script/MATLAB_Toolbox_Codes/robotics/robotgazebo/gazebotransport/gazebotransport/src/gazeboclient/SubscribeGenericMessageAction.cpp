/* Copyright 2019 The MathWorks, Inc. */

#include "gazebotransport/gazeboclient/SubscribeGenericMessageAction.hpp"
#include "gazebotransport/gazeboclient/checkErrorStatus.hpp"

namespace robotics {
namespace gazebotransport {
SubscribeGenericMessageAction::SubscribeGenericMessageAction()
    : m_message()
    , m_success(false) {
    // setup default subscribe message
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_SUBSCRIBE_IMAGE);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_subscribe_image()->set_topic_name("");

    // setup the generators
    m_generators[GenericMessageTypeStr[GenericMessageType::IMAGE]] =
        std::unique_ptr<ImageMessageHandler>(new ImageMessageHandler());
    m_generators[GenericMessageTypeStr[GenericMessageType::FUSED_IMU]] =
        std::unique_ptr<FusedIMUMessageHandler>(new FusedIMUMessageHandler());
    m_generators[GenericMessageTypeStr[GenericMessageType::LIDAR_SCAN]] =
        std::unique_ptr<LidarScanMessageHandler>(new LidarScanMessageHandler());
}

bool SubscribeGenericMessageAction::setTopic(std::string const& topicType,
                                             std::string const& topicName) {
    auto iter = m_generators.find(topicType);
    if (iter == m_generators.end()) {
        return false;
    } else {
        // fill the topic name
        iter->second->setupSubscribe(topicName, m_message);
        return true;
    }
}

std::string SubscribeGenericMessageAction::getMsgToSend() {
    return m_message.SerializeAsString();
}

void SubscribeGenericMessageAction::processMsg(boost::optional<std::string> const& msg) {
    m_success = false;

    auto status = checkErrorStatus(msg);

    if (status &&
        *status ==
            mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE) {
        m_success = true;
    }
}

bool SubscribeGenericMessageAction::success() const {
    return m_success;
}
} // namespace gazebotransport
} // namespace robotics
