/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/gazeboclient/StepSimulationAction.hpp"
#include "gazebotransport/gazeboclient/checkErrorStatus.hpp"

namespace robotics {
namespace gazebotransport {
StepSimulationAction::StepSimulationAction()
    : m_message()
    , m_success(false) {
    // setup default setp simulation message with step size 1
    m_message.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID_STEP_SIMULATION);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_step_simulation()->set_num_steps(1);
}

void StepSimulationAction::setStepSize(uint32_t stepSize) {
    m_message.mutable_step_simulation()->set_num_steps(stepSize);
}

std::string StepSimulationAction::getMsgToSend() {
    return m_message.SerializeAsString();
}

void StepSimulationAction::processMsg(boost::optional<std::string> const& msg) {
    m_success = false;

    auto status = checkErrorStatus(msg);

    if (status &&
        *status ==
            mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE) {
        m_success = true;
    }
}

bool StepSimulationAction::success() const {
    return m_success;
}
} // namespace gazebotransport
} // namespace robotics
