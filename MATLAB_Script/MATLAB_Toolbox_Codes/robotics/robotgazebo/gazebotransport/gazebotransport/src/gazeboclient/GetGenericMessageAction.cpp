/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/gazeboclient/GetGenericMessageAction.hpp"
#include "gazebotransport/gazeboclient/checkReplyIsNew.hpp"

namespace robotics {
namespace gazebotransport {
GetGenericMessageAction::GetGenericMessageAction()
    : m_request()
    , m_requestType("Image")
    , m_generators()
    , m_buffer(nullptr)
    , m_isNew(false) {
    // setup the request image message with empty topic name
    m_request.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_REQUEST_IMAGE);
    m_request.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_request.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_request.mutable_request_image()->set_topic_name("");

    // setup the generators
    m_generators[GenericMessageTypeStr[GenericMessageType::IMAGE]] =
        std::unique_ptr<ImageMessageHandler>(new ImageMessageHandler());
    m_generators[GenericMessageTypeStr[GenericMessageType::FUSED_IMU]] =
        std::unique_ptr<FusedIMUMessageHandler>(new FusedIMUMessageHandler());
    m_generators[GenericMessageTypeStr[GenericMessageType::LIDAR_SCAN]] =
        std::unique_ptr<LidarScanMessageHandler>(new LidarScanMessageHandler());
}

void GetGenericMessageAction::setTopic(std::string const& topicType, std::string const& topicName) {
    m_requestType = topicType;
    m_generators[m_requestType]->setupRequest(topicName, m_request);
}

void GetGenericMessageAction::setSimulationTime(uint64_t seconds, uint64_t nanoSeconds) {
    m_request.mutable_header()->mutable_time_stamp()->set_seconds(seconds);
    m_request.mutable_header()->mutable_time_stamp()->set_nano_seconds(nanoSeconds);
}

std::string GetGenericMessageAction::getMsgToSend() {
    return m_request.SerializeAsString();
}

void GetGenericMessageAction::processMsg(boost::optional<std::string> const& msg) {
    // Try fill the buffer by parsing the incoming message
    if (msg) {
        auto reply = std::make_shared<mw::internal::robotics::gazebotransport::Packet>();
        if (reply->ParseFromString(*msg) && m_generators[m_requestType]->matchRequest(*reply)) {
            m_isNew = !m_buffer || checkReplyIsNew(*m_buffer, *reply);
            m_buffer = reply;
        } else {
            m_buffer = nullptr;
            m_isNew = false;
        }
    } else {
        m_buffer = nullptr;
        m_isNew = false;
    }

    // Return zero initialized message if the buffer is not filled
    if (!m_buffer) {
        m_buffer = m_generators[m_requestType]->initalizeMessage(
            std::make_shared<mw::internal::robotics::gazebotransport::Packet>());
    }
}

std::pair<bool, std::shared_ptr<mw::internal::robotics::gazebotransport::Packet>>
GetGenericMessageAction::getMessage() const {
    return std::make_pair(m_isNew, m_buffer);
}
} // namespace gazebotransport
} // namespace robotics
