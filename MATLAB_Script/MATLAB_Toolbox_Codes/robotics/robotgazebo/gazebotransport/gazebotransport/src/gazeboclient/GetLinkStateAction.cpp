/* Copyright 2020 The MathWorks, Inc. */
#include "gazebotransport/gazeboclient/GetLinkStateAction.hpp"
#include "gazebotransport/gazeboclient/checkReplyIsNew.hpp"

namespace robotics {
namespace gazebotransport {
GetLinkStateAction::GetLinkStateAction()
    : m_isNew(false)
    , m_message()
    , m_linkState(nullptr) {
    // setup default step simulation message with step size 1
    m_message.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                           PacketHeader_MsgID_GET_LINK_STATE);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_get_link_state()->set_model_name("");
    m_message.mutable_get_link_state()->set_link_name("");
}

void GetLinkStateAction::setLinkName(std::string const& modelName, std::string const& linkName) {
    m_message.mutable_get_link_state()->set_model_name(modelName);
    m_message.mutable_get_link_state()->set_link_name(linkName);
}

void GetLinkStateAction::setSimulationTime(uint64_t seconds, uint64_t nanoSeconds) {
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(seconds);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(nanoSeconds);
}

std::string GetLinkStateAction::getMsgToSend() {
    return m_message.SerializeAsString();
}

std::pair<bool, std::shared_ptr<mw::internal::robotics::gazebotransport::Packet>>
GetLinkStateAction::getLastLinkState() const {
    return std::make_pair(m_isNew, m_linkState);
}

void GetLinkStateAction::processMsg(boost::optional<std::string> const& msg) {
    if (msg) {
        auto reply = std::make_shared<mw::internal::robotics::gazebotransport::Packet>();
        if (reply->ParseFromString(*msg) && reply->has_link_state()) {
            // if the link state is not previously stored, the reply must be new
            // if the link state is stored, check whether it is new by comparing to the previous
            // message
            m_isNew = !m_linkState || checkReplyIsNew(*m_linkState, *reply);
            // store the link state message
            m_linkState = reply;
        } else {
            m_isNew = false;
            m_linkState = nullptr;
        }
    } else {
        m_isNew = false;
        m_linkState = nullptr;
    }
}
} // namespace gazebotransport
} // namespace robotics
