/* Copyright 2019 The MathWorks, Inc. */
#include "ServerTestMsgHandler.hpp"

namespace robotics {
namespace gazebotransport {
ServerTestMsgHandler::ServerTestMsgHandler() {
}

std::string ServerTestMsgHandler::handleMessage(
    mw::internal::robotics::gazebotransport::Packet const& msgContent) {
    mw::internal::robotics::gazebotransport::Packet replyMsg;

    if (msgContent.request_image().has_topic_name()) {

        replyMsg.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID_STATUS);
        replyMsg.mutable_header()->mutable_time_stamp()->set_seconds(0);
        replyMsg.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        replyMsg.set_status(mw::internal::robotics::gazebotransport::Packet_CoSimError_NONE);
    } else {
        replyMsg.mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID_STATUS);
        replyMsg.mutable_header()->mutable_time_stamp()->set_seconds(0);
        replyMsg.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        replyMsg.set_status(mw::internal::robotics::gazebotransport::Packet_CoSimError_MSG_INVALID);
    }

    // Returns success message
    return replyMsg.SerializeAsString();
}

uint32_t ServerTestMsgHandler::getAcceptID() const {
    // Message ID from MsgTable
    return mw::internal::robotics::gazebotransport::PacketHeader_MsgID_REQUEST_IMAGE;
}
} // namespace gazebotransport
} // namespace robotics
