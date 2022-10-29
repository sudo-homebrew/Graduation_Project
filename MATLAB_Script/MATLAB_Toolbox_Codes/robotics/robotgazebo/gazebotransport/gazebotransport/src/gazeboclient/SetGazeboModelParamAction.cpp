/* Copyright 2020 The MathWorks, Inc. */

#include "gazebotransport/gazeboclient/SetGazeboModelParamAction.hpp"
#include "gazebotransport/gazeboclient/checkErrorStatus.hpp"
#include "gazebotransport/gazeboclient/checkErrorMessage.hpp"

namespace robotics {
namespace gazebotransport {
SetGazeboModelParamAction::SetGazeboModelParamAction()
    : m_message()
    , m_success(false) {
    // setup default set gazebo model command
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_SET_GAZEBO_MODEL_PARAM);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_gazebo_model()->set_name("");
}

std::string SetGazeboModelParamAction::getMsgToSend() {
    return m_message.SerializeAsString();
}

void SetGazeboModelParamAction::processMsg(boost::optional<std::string> const& msg) {

    m_success = 1; // default error

    auto status = checkErrorStatus(msg);

    // several message error types handled on MATLAB side
    if (status) {
        switch (*status) {
        case mw::internal::robotics::gazebotransport::Packet_CoSimError::
            Packet_CoSimError_NONE: // No error
        {
            m_success = 0;
        } break;
        case mw::internal::robotics::gazebotransport::Packet_CoSimError::
            Packet_CoSimError_MODEL_NAME_INVALID: // invalid model name
        {
            m_success = 1;
        } break;
        case mw::internal::robotics::gazebotransport::Packet_CoSimError::
            Packet_CoSimError_LINK_NAME_INVALID: // invalid link name
        {
            m_success = 2;
        } break;
        case mw::internal::robotics::gazebotransport::Packet_CoSimError::
            Packet_CoSimError_JOINT_NAME_INVALID: // invalid joint name
        {
            m_success = 3;
        } break;
        case mw::internal::robotics::gazebotransport::Packet_CoSimError::
            Packet_CoSimError_JOINT_AXIS_NONE: // joint with no axis
        {
            m_success = 4;
        } break;
        case mw::internal::robotics::gazebotransport::Packet_CoSimError::
            Packet_CoSimError_INVALID_JOINT_AXIS: // invalid axis index
        {
            m_success = 5;
        } break;
        default:
            break;
        }
    }
    // get error message
    m_error_message = checkErrorMessage(msg);
}

void SetGazeboModelParamAction::setGazeboModelParam(
    mw::internal::robotics::gazebotransport::Gazebomodel& message) {
    m_message.mutable_gazebo_model()->MergeFrom(message);
}

std::pair<uint8_t, std::string> SetGazeboModelParamAction::success() const {
    return std::make_pair(m_success, m_error_message);
}
} // namespace gazebotransport
} // namespace robotics
