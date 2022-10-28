/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/gazeboclient/GetMaxStepSizeAction.hpp"
#include <limits>

namespace robotics {

namespace gazebotransport {

GetMaxStepSizeAction::GetMaxStepSizeAction()
    : m_message()
    , m_maxStepSize(std::numeric_limits<double>::quiet_NaN()) {
    // setup get max step size message
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_MAX_STEP_SIZE);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_max_step_size()->set_type(
        mw::internal::robotics::gazebotransport::MaxStepSize_TYPE::MaxStepSize_TYPE_GET_STEP_SIZE);
    m_message.mutable_max_step_size()->set_size(0);
}

std::string GetMaxStepSizeAction::getMsgToSend() {
    return m_message.SerializeAsString();
}

void GetMaxStepSizeAction::processMsg(boost::optional<std::string> const& msg) {
    // initialize max step size to nan
    m_maxStepSize = std::numeric_limits<double>::quiet_NaN();
    // try to update the max step size according to the new message
    if (msg) {
        mw::internal::robotics::gazebotransport::Packet reply;
        if (reply.ParseFromString(*msg) && reply.has_max_step_size()) {
            m_maxStepSize = reply.max_step_size().size();
        }
    }
}

double GetMaxStepSizeAction::getMaxStepSize() {
    return m_maxStepSize;
}
} // namespace gazebotransport
} // namespace robotics
