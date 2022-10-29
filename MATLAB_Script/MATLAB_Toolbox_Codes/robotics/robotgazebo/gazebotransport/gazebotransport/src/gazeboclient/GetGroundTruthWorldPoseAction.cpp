/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/gazeboclient/GetGroundTruthWorldPoseAction.hpp"
#include "gazebotransport/gazeboclient/checkReplyIsNew.hpp"

namespace robotics {
namespace gazebotransport {
GetGroundTruthWorldPoseAction::GetGroundTruthWorldPoseAction()
    : m_isNew(false)
    , m_message()
    , m_pose(nullptr) {
    // setup default step simulation message with step size 1
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_GET_GROUND_TRUTH_WORLD_POSE);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_get_ground_truth_world_pose()->set_model_name("");
    m_message.mutable_get_ground_truth_world_pose()->set_link_name("");
}

void GetGroundTruthWorldPoseAction::setLinkName(std::string const& modelName,
                                                std::string const& linkName) {
    m_message.mutable_get_ground_truth_world_pose()->set_model_name(modelName);
    m_message.mutable_get_ground_truth_world_pose()->set_link_name(linkName);
}

void GetGroundTruthWorldPoseAction::setSimulationTime(uint64_t seconds, uint64_t nanoSeconds) {
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(seconds);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(nanoSeconds);
}

std::string GetGroundTruthWorldPoseAction::getMsgToSend() {
    return m_message.SerializeAsString();
}

std::pair<bool, std::shared_ptr<mw::internal::robotics::gazebotransport::Packet>>
GetGroundTruthWorldPoseAction::getLastPose() const {
    return std::make_pair(m_isNew, m_pose);
}

void GetGroundTruthWorldPoseAction::processMsg(boost::optional<std::string> const& msg) {
    if (msg) {
        auto reply = std::make_shared<mw::internal::robotics::gazebotransport::Packet>();
        if (reply->ParseFromString(*msg) && reply->has_pose()) {
            // if the pose is not previously stored, the reply must be new
            // if the pose is stored, check whether it is new by comparing to the previous message
            m_isNew = !m_pose || checkReplyIsNew(*m_pose, *reply);
            // store the pose message
            m_pose = reply;
        } else {
            m_isNew = false;
            m_pose = nullptr;
        }
    } else {
        m_isNew = false;
        m_pose = nullptr;
    }
}
} // namespace gazebotransport
} // namespace robotics
