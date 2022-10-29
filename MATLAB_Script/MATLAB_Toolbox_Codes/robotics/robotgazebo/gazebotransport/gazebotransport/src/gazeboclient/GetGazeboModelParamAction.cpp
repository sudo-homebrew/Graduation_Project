/* Copyright 2020 The MathWorks, Inc. */

#include "gazebotransport/gazeboclient/GetGazeboModelParamAction.hpp"
#include "gazebotransport/gazeboclient/checkErrorStatus.hpp"

namespace robotics {
namespace gazebotransport {
GetGazeboModelParamAction::GetGazeboModelParamAction()
    : m_isNew(false)
    , m_message()
    , m_gazeboModelParam(nullptr) {
    // setup default get gazebo model param command
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_GET_GAZEBO_MODEL_PARAM);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_get_gazebo_model_param()->set_model_name("");
}

std::string GetGazeboModelParamAction::getMsgToSend() {
    return m_message.SerializeAsString();
}

void GetGazeboModelParamAction::setSimulationTime(uint64_t seconds, uint64_t nanoSeconds) {
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(seconds);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(nanoSeconds);
}

void GetGazeboModelParamAction::setModelName(std::string const& modelName,
                                             bool isLink,
                                             std::string const& linkJointName) {
    m_message.mutable_get_gazebo_model_param()->set_model_name(modelName);
    if (strcmp(linkJointName.c_str(), "")) {
        // setup optional fields if link or joint parameter requested
        m_message.mutable_get_gazebo_model_param()->set_is_link(isLink);
        m_message.mutable_get_gazebo_model_param()->set_link_joint_name(linkJointName);
    }
}

void GetGazeboModelParamAction::processMsg(boost::optional<std::string> const& msg) {
    if (msg) {
        auto reply = std::make_shared<mw::internal::robotics::gazebotransport::Packet>();
        if (reply->ParseFromString(*msg) && reply->has_gazebo_model()) {
            // if the gazebo model param is not previously stored, the reply must be new
            // if the gazebo model param is stored, check whether it is new by comparing to the
            // previous message
            m_isNew = !m_gazeboModelParam;
            // store the gazebo model param message
            m_gazeboModelParam = reply;
        } else {
            m_isNew = false;
            m_gazeboModelParam = nullptr;
        }
    } else {
        m_isNew = false;
        m_gazeboModelParam = nullptr;
    }
}

std::pair<bool, std::shared_ptr<mw::internal::robotics::gazebotransport::Packet>>
GetGazeboModelParamAction::getGazeboModelParam() const {
    return std::make_pair(m_isNew, m_gazeboModelParam);
}
} // namespace gazebotransport
} // namespace robotics
