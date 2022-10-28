/* Copyright 2020 The MathWorks, Inc. */
#include "gazebotransport/gazeboclient/GetJointStateAction.hpp"
#include "gazebotransport/gazeboclient/checkReplyIsNew.hpp"

namespace robotics {
namespace gazebotransport {
GetJointStateAction::GetJointStateAction()
    : m_isNew(false)
    , m_message()
    , m_jointState(nullptr) {
    // setup default step simulation message with step size 1
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_GET_JOINT_STATE);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_get_joint_state()->set_model_name("");
    m_message.mutable_get_joint_state()->set_joint_name("");
}

void GetJointStateAction::setJointName(std::string const& modelName, std::string const& jointName) {
    m_message.mutable_get_joint_state()->set_model_name(modelName);
    m_message.mutable_get_joint_state()->set_joint_name(jointName);
}

void GetJointStateAction::setSimulationTime(uint64_t seconds, uint64_t nanoSeconds) {
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(seconds);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(nanoSeconds);
}

std::string GetJointStateAction::getMsgToSend() {
    return m_message.SerializeAsString();
}

std::pair<bool, std::shared_ptr<mw::internal::robotics::gazebotransport::Packet>>
GetJointStateAction::getLastJointState() const {
    return std::make_pair(m_isNew, m_jointState);
}

void GetJointStateAction::processMsg(boost::optional<std::string> const& msg) {
    if (msg) {
        auto reply = std::make_shared<mw::internal::robotics::gazebotransport::Packet>();
        if (reply->ParseFromString(*msg) && reply->has_joint_state()) {
            // if the joint state is not previously stored, the reply must be new
            // if the joint state is stored, check whether it is new by comparing to the previous
            // message
            m_isNew = !m_jointState || checkReplyIsNew(*m_jointState, *reply);
            // store the joint state message
            m_jointState = reply;

        } else {
            m_isNew = false;
            m_jointState = nullptr;
        }
    } else {
        m_isNew = false;
        m_jointState = nullptr;
    }
}
} // namespace gazebotransport
} // namespace robotics
