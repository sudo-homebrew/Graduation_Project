/* Copyright 2020 The MathWorks, Inc. */

#include "gazebotransport/gazeboclient/SetLinkWorldPoseAction.hpp"
#include "gazebotransport/gazeboclient/checkErrorStatus.hpp"

namespace robotics {
namespace gazebotransport {
SetLinkWorldPoseAction::SetLinkWorldPoseAction()
    : m_message()
    , m_success(false) {
    // setup default set link world pose command
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_SET_LINK_WORLD_POSE);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_set_link_world_pose()->set_model_name("");
    m_message.mutable_set_link_world_pose()->set_link_name("");
    m_message.mutable_set_link_world_pose()->mutable_pose()->mutable_position()->set_x(0);
    m_message.mutable_set_link_world_pose()->mutable_pose()->mutable_position()->set_y(0);
    m_message.mutable_set_link_world_pose()->mutable_pose()->mutable_position()->set_z(0);
    m_message.mutable_set_link_world_pose()->mutable_pose()->mutable_orientation()->set_w(0);
    m_message.mutable_set_link_world_pose()->mutable_pose()->mutable_orientation()->set_x(0);
    m_message.mutable_set_link_world_pose()->mutable_pose()->mutable_orientation()->set_y(0);
    m_message.mutable_set_link_world_pose()->mutable_pose()->mutable_orientation()->set_z(0);
    m_message.mutable_set_link_world_pose()->mutable_duration()->set_seconds(0);
    m_message.mutable_set_link_world_pose()->mutable_duration()->set_nano_seconds(0);
}

std::string SetLinkWorldPoseAction::getMsgToSend() {
    return m_message.SerializeAsString();
}

void SetLinkWorldPoseAction::processMsg(boost::optional<std::string> const& msg) {
    m_success = false;

    auto status = checkErrorStatus(msg);

    if (status &&
        *status ==
            mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE) {
        m_success = true;
    }
}

void SetLinkWorldPoseAction::setLinkWorldPose(std::string const& modelName,
                                              std::string const& linkName,
                                              std::vector<double> const& pose,
                                              uint64_t durationSeconds,
                                              uint64_t durationNanoSeconds) {
    m_message.mutable_set_link_world_pose()->set_model_name(modelName);
    m_message.mutable_set_link_world_pose()->set_link_name(linkName);
    m_message.mutable_set_link_world_pose()->mutable_pose()->mutable_position()->set_x(pose[0]);
    m_message.mutable_set_link_world_pose()->mutable_pose()->mutable_position()->set_y(pose[1]);
    m_message.mutable_set_link_world_pose()->mutable_pose()->mutable_position()->set_z(pose[2]);
    m_message.mutable_set_link_world_pose()->mutable_pose()->mutable_orientation()->set_w(pose[6]);
    m_message.mutable_set_link_world_pose()->mutable_pose()->mutable_orientation()->set_x(pose[3]);
    m_message.mutable_set_link_world_pose()->mutable_pose()->mutable_orientation()->set_y(pose[4]);
    m_message.mutable_set_link_world_pose()->mutable_pose()->mutable_orientation()->set_z(pose[5]);
    m_message.mutable_set_link_world_pose()->mutable_duration()->set_seconds(durationSeconds);
    m_message.mutable_set_link_world_pose()->mutable_duration()->set_nano_seconds(
        durationNanoSeconds);
}

bool SetLinkWorldPoseAction::success() const {
    return m_success;
}
} // namespace gazebotransport
} // namespace robotics
