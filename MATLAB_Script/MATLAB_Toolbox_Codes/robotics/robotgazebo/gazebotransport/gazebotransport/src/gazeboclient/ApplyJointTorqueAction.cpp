/* Copyright 2019 The MathWorks, Inc. */

#include "gazebotransport/gazeboclient/ApplyJointTorqueAction.hpp"
#include "gazebotransport/gazeboclient/checkErrorStatus.hpp"

namespace robotics {
namespace gazebotransport {
ApplyJointTorqueAction::ApplyJointTorqueAction()
    : m_message()
    , m_success(false) {
    // setup default apply joint torque command
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_APPLY_JOINT_TORQUE);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_apply_joint_torque()->set_model_name("");
    m_message.mutable_apply_joint_torque()->set_joint_name("");
    m_message.mutable_apply_joint_torque()->set_index(0);
    m_message.mutable_apply_joint_torque()->set_effort(0);
    m_message.mutable_apply_joint_torque()->mutable_duration()->set_seconds(0);
    m_message.mutable_apply_joint_torque()->mutable_duration()->set_nano_seconds(0);
}

std::string ApplyJointTorqueAction::getMsgToSend() {
    return m_message.SerializeAsString();
}

void ApplyJointTorqueAction::processMsg(boost::optional<std::string> const& msg) {
    m_success = false;

    auto status = checkErrorStatus(msg);

    if (status &&
        *status ==
            mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE) {
        m_success = true;
    }
}

void ApplyJointTorqueAction::setJointTorque(std::string const& modelName,
                                            std::string const& jointName,
                                            uint32_t indexValue,
                                            double effortValue,
                                            uint64_t durationSeconds,
                                            uint64_t durationNanoSeconds) {
    m_message.mutable_apply_joint_torque()->set_model_name(modelName);
    m_message.mutable_apply_joint_torque()->set_joint_name(jointName);
    m_message.mutable_apply_joint_torque()->set_index(indexValue);
    m_message.mutable_apply_joint_torque()->set_effort(effortValue);
    m_message.mutable_apply_joint_torque()->mutable_duration()->set_seconds(durationSeconds);
    m_message.mutable_apply_joint_torque()->mutable_duration()->set_nano_seconds(
        durationNanoSeconds);
}

bool ApplyJointTorqueAction::success() const {
    return m_success;
}
} // namespace gazebotransport
} // namespace robotics
