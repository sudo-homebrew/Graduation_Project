/* Copyright 2020 The MathWorks, Inc. */

#include "gazebotransport/gazeboclient/SetJointPositionAction.hpp"
#include "gazebotransport/gazeboclient/checkErrorStatus.hpp"

namespace robotics {
namespace gazebotransport {
SetJointPositionAction::SetJointPositionAction()
    : m_message()
    , m_success(false) {
    // setup default set joint position command
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_SET_JOINT_POSITION);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_set_joint_position()->set_model_name("");
    m_message.mutable_set_joint_position()->set_joint_name("");
    m_message.mutable_set_joint_position()->set_index(0);
    m_message.mutable_set_joint_position()->set_position(0);
    m_message.mutable_set_joint_position()->mutable_duration()->set_seconds(0);
    m_message.mutable_set_joint_position()->mutable_duration()->set_nano_seconds(0);
}

std::string SetJointPositionAction::getMsgToSend() {
    return m_message.SerializeAsString();
}

void SetJointPositionAction::processMsg(boost::optional<std::string> const& msg) {
    m_success = false;

    auto status = checkErrorStatus(msg);

    if (status &&
        *status ==
            mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE) {
        m_success = true;
    }
}

void SetJointPositionAction::setJointPosition(std::string const& modelName,
                                              std::string const& jointName,
                                              uint32_t indexValue,
                                              double position,
                                              uint64_t durationSeconds,
                                              uint64_t durationNanoSeconds) {
    m_message.mutable_set_joint_position()->set_model_name(modelName);
    m_message.mutable_set_joint_position()->set_joint_name(jointName);
    m_message.mutable_set_joint_position()->set_index(indexValue);
    m_message.mutable_set_joint_position()->set_position(position);
    m_message.mutable_set_joint_position()->mutable_duration()->set_seconds(durationSeconds);
    m_message.mutable_set_joint_position()->mutable_duration()->set_nano_seconds(
        durationNanoSeconds);
}

bool SetJointPositionAction::success() const {
    return m_success;
}
} // namespace gazebotransport
} // namespace robotics
