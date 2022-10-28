/* Copyright 2019 The MathWorks, Inc. */
#include "TestCustomMsgHandler.hpp"

using namespace mw::internal::robotics::gazebotransport;

namespace robotics {
namespace gazebotransport {
TestCustomMsgHandler::TestCustomMsgHandler() {
}

/// Destructor
TestCustomMsgHandler::~TestCustomMsgHandler() {
}

std::string TestCustomMsgHandler::initPublisher(Packet const& msgContent) {
    Packet replyMsg;

    replyMsg.mutable_header()->set_id(PacketHeader_MsgID_STATUS);
    replyMsg.mutable_header()->mutable_time_stamp()->set_seconds(0);
    replyMsg.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    replyMsg.set_status(Packet_CoSimError_NONE);

    // Returns success message
    return replyMsg.SerializeAsString();
}

std::string TestCustomMsgHandler::initSubscriber(Packet const& msgContent) {
    Packet replyMsg;

    replyMsg.mutable_header()->set_id(PacketHeader_MsgID_STATUS);
    replyMsg.mutable_header()->mutable_time_stamp()->set_seconds(0);
    replyMsg.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    replyMsg.set_status(Packet_CoSimError_NONE);

    // Returns success message
    return replyMsg.SerializeAsString();
}

std::string TestCustomMsgHandler::publishCustomMsg(Packet const& msgContent) {
    Packet replyMsg;

    replyMsg.mutable_header()->set_id(PacketHeader_MsgID_STATUS);
    replyMsg.mutable_header()->mutable_time_stamp()->set_seconds(0);
    replyMsg.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    replyMsg.set_status(Packet_CoSimError_NONE);

    // Returns success message
    return replyMsg.SerializeAsString();
}

std::pair<bool, std::string> TestCustomMsgHandler::subscribeCustomMsg(Packet const& msgContent) {
    Packet replyMsg;
    replyMsg.mutable_header()->set_id(PacketHeader_MsgID_IMAGE);
    replyMsg.mutable_header()->mutable_time_stamp()->set_seconds(0);
    replyMsg.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    replyMsg.mutable_image()->set_data("128");
    replyMsg.mutable_image()->set_width(320);
    replyMsg.mutable_image()->set_height(240);
    replyMsg.mutable_image()->set_data_type("uint8");

    // Returns success message
    return std::make_pair(true, replyMsg.SerializeAsString());
}

void TestCustomMsgHandler::resetInternal() {
}

} // namespace gazebotransport
} // namespace robotics
