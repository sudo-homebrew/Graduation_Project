/* Copyright 2020 The MathWorks, Inc. */

#include "gazebotransport/gazeboclient/SetLinkAngularVelocityAction.hpp"
#include "gazebotransport/gazeboclient/checkErrorStatus.hpp"

namespace robotics {
namespace gazebotransport {
SetLinkAngularVelocityAction::SetLinkAngularVelocityAction()
    : m_message()
    , m_success(false) {
    // setup default set link angular velocity command
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_SET_LINK_ANGULAR_VELOCITY);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_set_link_angular_velocity()->set_model_name("");
    m_message.mutable_set_link_angular_velocity()->set_link_name("");
    m_message.mutable_set_link_angular_velocity()->mutable_velocity()->set_x(0);
    m_message.mutable_set_link_angular_velocity()->mutable_velocity()->set_y(0);
    m_message.mutable_set_link_angular_velocity()->mutable_velocity()->set_z(0);
    m_message.mutable_set_link_angular_velocity()->mutable_duration()->set_seconds(0);
    m_message.mutable_set_link_angular_velocity()->mutable_duration()->set_nano_seconds(0);
}

std::string SetLinkAngularVelocityAction::getMsgToSend() {
    return m_message.SerializeAsString();
}

void SetLinkAngularVelocityAction::processMsg(boost::optional<std::string> const& msg) {
    m_success = false;

    auto status = checkErrorStatus(msg);

    if (status &&
        *status ==
            mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE) {
        m_success = true;
    }
}

void SetLinkAngularVelocityAction::setLinkAngularVelocity(std::string const& modelName,
                                                          std::string const& linkName,
                                                          std::vector<double> const& velocity,
                                                          uint64_t durationSeconds,
                                                          uint64_t durationNanoSeconds) {
    m_message.mutable_set_link_angular_velocity()->set_model_name(modelName);
    m_message.mutable_set_link_angular_velocity()->set_link_name(linkName);
    m_message.mutable_set_link_angular_velocity()->mutable_velocity()->set_x(velocity[0]);
    m_message.mutable_set_link_angular_velocity()->mutable_velocity()->set_y(velocity[1]);
    m_message.mutable_set_link_angular_velocity()->mutable_velocity()->set_z(velocity[2]);
    m_message.mutable_set_link_angular_velocity()->mutable_duration()->set_seconds(durationSeconds);
    m_message.mutable_set_link_angular_velocity()->mutable_duration()->set_nano_seconds(
        durationNanoSeconds);
}

bool SetLinkAngularVelocityAction::success() const {
    return m_success;
}
} // namespace gazebotransport
} // namespace robotics