/* Copyright 2019 The MathWorks, Inc. */

#include "gazebotransport/gazeboclient/StopCoSimAction.hpp"
#include "gazebotransport/gazeboclient/checkErrorStatus.hpp"

namespace robotics {
namespace gazebotransport {
StopCoSimAction::StopCoSimAction()
    : m_message()
    , m_result(false) {
    // setup default setp simulation message with step size 1
    m_message.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_STOP_COSIM);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_stop_cosim()->set_client_id("");
}

void StopCoSimAction::setClientID(std::string const& clientID) {
    m_message.mutable_stop_cosim()->set_client_id(clientID);
}

std::string StopCoSimAction::getMsgToSend() {
    return m_message.SerializeAsString();
}

void StopCoSimAction::processMsg(boost::optional<std::string> const& msg) {
    m_result = false;

    auto status = checkErrorStatus(msg);

    if (status &&
        *status ==
            mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE) {
        m_result = true;
    }
}

bool StopCoSimAction::success() const {
    return m_result;
}
} // namespace gazebotransport
} // namespace robotics
