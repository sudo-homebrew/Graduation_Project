/* Copyright 2019 The MathWorks, Inc. */

#include "gazebotransport/gazeboclient/ResetSimulationAction.hpp"
#include "gazebotransport/gazeboclient/checkErrorStatus.hpp"

namespace robotics {
namespace gazebotransport {
ResetSimulationAction::ResetSimulationAction()
    : m_message()
    , m_success(false) {
    // setup default setp simulation message with step size 1
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_RESET_SIMULATION);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_reset_simulation()->set_behavior(
        mw::internal::robotics::gazebotransport::ResetSimulation_ResetBehavior_RESET_TIME);
}

void ResetSimulationAction::setResetMode(
    mw::internal::robotics::gazebotransport::ResetSimulation_ResetBehavior resetMode) {
    m_message.mutable_reset_simulation()->set_behavior(resetMode);
}

std::string ResetSimulationAction::getMsgToSend() {
    return m_message.SerializeAsString();
}

void ResetSimulationAction::processMsg(boost::optional<std::string> const& msg) {
    m_success = false;

    auto status = checkErrorStatus(msg);

    if (status &&
        *status ==
            mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE) {
        m_success = true;
    }
}

bool ResetSimulationAction::success() const {
    return m_success;
}
} // namespace gazebotransport
} // namespace robotics
