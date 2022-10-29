/* Copyright 2021 The MathWorks, Inc. */

#include "gazebotransport/gazeboclient/GetGazeboModelSDFAction.hpp"
#include "gazebotransport/gazeboclient/checkErrorStatus.hpp"

namespace robotics {
namespace gazebotransport {
GetGazeboModelSDFAction::GetGazeboModelSDFAction()
    : m_isNew(false)
    , m_message()
    , m_gazeboModelSDF(nullptr) {
    // setup default get gazebo model SDF command
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_GET_GAZEBO_MODEL_SDF);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_get_gazebo_model_sdf()->set_model_name("");
}

std::string GetGazeboModelSDFAction::getMsgToSend() {
    return m_message.SerializeAsString();
}

void GetGazeboModelSDFAction::setSimulationTime(uint64_t seconds, uint64_t nanoSeconds) {
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(seconds);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(nanoSeconds);
}

void GetGazeboModelSDFAction::setModelName(std::string const& modelName) {
    m_message.mutable_get_gazebo_model_sdf()->set_model_name(modelName);
}

void GetGazeboModelSDFAction::processMsg(boost::optional<std::string> const& msg) {
    if (msg) {
        auto reply = std::make_shared<mw::internal::robotics::gazebotransport::Packet>();
        if (reply->ParseFromString(*msg) && reply->has_gazebo_model_sdf()) {
            // if the gazebo model SDF is not previously stored, the reply must be new
            // if the gazebo model SDF is stored, check whether it is new by comparing to the
            // previous message
            m_isNew = !m_gazeboModelSDF;
            // store the gazebo model SDF message
            m_gazeboModelSDF = reply;
        } else {
            m_isNew = false;
            m_gazeboModelSDF = nullptr;
        }
    } else {
        m_isNew = false;
        m_gazeboModelSDF = nullptr;
    }
}

std::pair<bool, std::shared_ptr<mw::internal::robotics::gazebotransport::Packet>>
GetGazeboModelSDFAction::getGazeboModelSDF() const {
    return std::make_pair(m_isNew, m_gazeboModelSDF);
}
} // namespace gazebotransport
} // namespace robotics
