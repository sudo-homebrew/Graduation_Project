/* Copyright 2019 The MathWorks, Inc. */

#include "gazebotransport/gazeboclient/ApplyLinkWrenchAction.hpp"
#include "gazebotransport/gazeboclient/checkErrorStatus.hpp"

namespace robotics {
namespace gazebotransport {
ApplyLinkWrenchAction::ApplyLinkWrenchAction()
    : m_message()
    , m_success(false) {
    // setup default apply joint torque command
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_APPLY_LINK_WRENCH);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_apply_link_wrench()->set_model_name("");
    m_message.mutable_apply_link_wrench()->set_link_name("");
    m_message.mutable_apply_link_wrench()->set_force_type("");
    m_message.mutable_apply_link_wrench()->set_fx(0);
    m_message.mutable_apply_link_wrench()->set_fy(0);
    m_message.mutable_apply_link_wrench()->set_fz(0);
    m_message.mutable_apply_link_wrench()->set_torque_type("");
    m_message.mutable_apply_link_wrench()->set_tx(0);
    m_message.mutable_apply_link_wrench()->set_ty(0);
    m_message.mutable_apply_link_wrench()->set_tz(0);
    m_message.mutable_apply_link_wrench()->mutable_duration()->set_seconds(0);
    m_message.mutable_apply_link_wrench()->mutable_duration()->set_nano_seconds(0);
}

std::string ApplyLinkWrenchAction::getMsgToSend() {
    return m_message.SerializeAsString();
}

void ApplyLinkWrenchAction::processMsg(boost::optional<std::string> const& msg) {
    m_success = false;

    auto status = checkErrorStatus(msg);

    if (status &&
        *status ==
            mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE) {
        m_success = true;
    }
}

void ApplyLinkWrenchAction::setLinkWrench(std::string const& modelName,
                                          std::string const& linkName,
                                          std::string const& forceType,
                                          std::vector<double> const& force,
                                          std::string const& torqueType,
                                          std::vector<double> const& torque,
                                          uint64_t durationSeconds,
                                          uint64_t durationNanoSeconds) {
    m_message.mutable_apply_link_wrench()->set_model_name(modelName);
    m_message.mutable_apply_link_wrench()->set_link_name(linkName);
    m_message.mutable_apply_link_wrench()->set_force_type(forceType);
    m_message.mutable_apply_link_wrench()->set_fx(force[0]);
    m_message.mutable_apply_link_wrench()->set_fy(force[1]);
    m_message.mutable_apply_link_wrench()->set_fz(force[2]);
    m_message.mutable_apply_link_wrench()->set_torque_type(torqueType);
    m_message.mutable_apply_link_wrench()->set_tx(torque[0]);
    m_message.mutable_apply_link_wrench()->set_ty(torque[1]);
    m_message.mutable_apply_link_wrench()->set_tz(torque[2]);
    m_message.mutable_apply_link_wrench()->mutable_duration()->set_seconds(durationSeconds);
    m_message.mutable_apply_link_wrench()->mutable_duration()->set_nano_seconds(
        durationNanoSeconds);
}

bool ApplyLinkWrenchAction::success() const {
    return m_success;
}
} // namespace gazebotransport
} // namespace robotics
