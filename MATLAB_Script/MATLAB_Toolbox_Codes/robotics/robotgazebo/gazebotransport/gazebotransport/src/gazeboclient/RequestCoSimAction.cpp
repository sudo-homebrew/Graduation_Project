/* Copyright 2019 The MathWorks, Inc. */

#include "gazebotransport/gazeboclient/RequestCoSimAction.hpp"
#include "gazebotransport/gazeboclient/checkErrorStatus.hpp"

namespace robotics {
namespace gazebotransport {
RequestCoSimAction::RequestCoSimAction()
    : m_message()
    , m_result(false) {
    // setup default step simulation message with step size 1
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_REQUEST_COSIM);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_request_cosim()->set_client_id("");
    m_message.mutable_request_cosim()->set_duration(0);
}

void RequestCoSimAction::setRequest(std::string const& clientID, double duration) {
    m_message.mutable_request_cosim()->set_client_id(clientID);
    m_message.mutable_request_cosim()->set_duration(duration);
}

std::string RequestCoSimAction::getMsgToSend() {
    return m_message.SerializeAsString();
}

void RequestCoSimAction::processMsg(boost::optional<std::string> const& msg) {
    m_result = false;

    auto status = checkErrorStatus(msg);

    if (status &&
        *status ==
            mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE) {
        m_result = true;
    }
}

bool RequestCoSimAction::success() const {
    return m_result;
}
} // namespace gazebotransport
} // namespace robotics
