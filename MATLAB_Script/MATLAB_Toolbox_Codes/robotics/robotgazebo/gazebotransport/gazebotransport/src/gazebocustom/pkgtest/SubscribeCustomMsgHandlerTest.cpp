/* Copyright 2019 The MathWorks, Inc. */
#include "mw_gtest/gtest.hpp"

#include "gazebotransport/Client.hpp"

#include "gazebotransport/GazeboServer.hpp"
#include "TestCustomMsgHandler.hpp"
#include "gazebotransport/gazebocustom/CustomMsgDispatcher.hpp"
#include "gazebotransport/gazebocustom/gazebocustommsghandler/SubscribeCustomMsgHandler.hpp"

#include "gazebotransport/Server.hpp"
#include "gazebotransport/Callback.hpp"
#include "gazebotransport/PacketEnding.hpp"
#include "google/protobuf/util/message_differencer.h"

/// Test SubscribeCustomMsgHandler functionality
/// These tests are performed to validate message type for SubscribeCustomMsgHandler handler
class SubscribeCustomMsgHandlerTest : public testing::Test {
  public:
    std::shared_ptr<robotics::gazebotransport::GazeboServer> m_server;
    std::shared_ptr<robotics::gazebotransport::Client> m_client;
    /// create CustomMsgDispatcher object
    std::shared_ptr<robotics::gazebotransport::CustomMsgDispatcher> m_customDispatch;
    std::string ipAddress = "127.0.0.1";

    void SetUp() {
        m_server = std::make_shared<robotics::gazebotransport::GazeboServer>(
            static_cast<uint16_t>(std::stoi("0")));
        m_server->run();
        m_client = std::make_shared<robotics::gazebotransport::Client>(
            ipAddress, m_server->getPortName(), boost::posix_time::milliseconds(10000));

        m_customDispatch = std::make_shared<robotics::gazebotransport::CustomMsgDispatcher>();

        // register SubscribeCustomMsgHandler
        m_server->registerHandler(
            std::make_shared<robotics::gazebotransport::SubscribeCustomMsgHandler>(
                this->m_customDispatch));

        // register test custom message handler
        m_customDispatch->registerCustomHandler(
            std::make_shared<robotics::gazebotransport::TestCustomMsgHandler>(),
            "gazebo_msgs/test_custom");
    }

    void TearDown() {
        m_server->stop();
        m_client->shutdown();
        // reset handler
        m_customDispatch->reset();
    }
};

/// POSITIVE TEST
/// Test SubscribeCustomMsgHandler for valid message type
MWTEST_F(SubscribeCustomMsgHandlerTest, SubscribeCustomMsgHandlerTestValidMsgType) {

    // create Packet message with CUSTOM_MESSAGE_SUBSCRIBER type
    mw::internal::robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID_CUSTOM_MESSAGE_SUBSCRIBER);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_request_custom_message_support()->set_topic_name("~/custom_message");
    m_message.mutable_request_custom_message_support()->set_message_type("gazebo_msgs/test_custom");

    // serialize message
    auto msg = m_message.SerializeAsString();

    // send packet message & receive reply message over server-client interaction
    auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(10000));

    // deserialize message
    mw::internal::robotics::gazebotransport::Packet reply;
    if (replyMsg) {
        reply.ParseFromString(*replyMsg);
    }

    // deserialize custom message stored inside Packet message
    mw::internal::robotics::gazebotransport::Packet customReply;
    if (reply.has_custom_message_support()) {
        customReply.ParseFromString(reply.custom_message_support().data());
    }

    // expected reply message
    mw::internal::robotics::gazebotransport::Packet expectedReplyMsg;
    expectedReplyMsg.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID_IMAGE);
    expectedReplyMsg.mutable_header()->mutable_time_stamp()->set_seconds(0);
    expectedReplyMsg.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    expectedReplyMsg.mutable_image()->set_data("128");
    expectedReplyMsg.mutable_image()->set_width(320);
    expectedReplyMsg.mutable_image()->set_height(240);
    expectedReplyMsg.mutable_image()->set_data_type("uint8");

    // expected FALSE as invalid message type
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(customReply, expectedReplyMsg));
}

/// NEGATIVE TEST
/// Test SubscribeCustomMsgHandler for invalid message type
MWTEST_F(SubscribeCustomMsgHandlerTest, SubscribeCustomMsgHandlerTestInValidMsgType) {

    // create Packet message with CUSTOM_MESSAGE_SUBSCRIBER type
    mw::internal::robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID_CUSTOM_MESSAGE_SUBSCRIBER);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_request_custom_message_support()->set_topic_name("~/custom_message");
    m_message.mutable_request_custom_message_support()->set_message_type(
        "gazebo_msgs/test_incorrect");

    // serialize message
    auto msg = m_message.SerializeAsString();

    // send packet message & receive reply message over server-client interaction
    auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(10000));

    // deserialize message
    mw::internal::robotics::gazebotransport::Packet reply;
    if (replyMsg) {
        reply.ParseFromString(*replyMsg);
    }

    // deserialize custom message stored inside Packet message
    mw::internal::robotics::gazebotransport::Packet customReply;
    if (reply.has_custom_message_support()) {
        customReply.ParseFromString(reply.custom_message_support().data());
    }

    // expected reply message
    mw::internal::robotics::gazebotransport::Packet expectedReplyMsg;
    expectedReplyMsg.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID_IMAGE);
    expectedReplyMsg.mutable_header()->mutable_time_stamp()->set_seconds(0);
    expectedReplyMsg.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    expectedReplyMsg.mutable_image()->set_data("128");
    expectedReplyMsg.mutable_image()->set_width(320);
    expectedReplyMsg.mutable_image()->set_height(240);
    expectedReplyMsg.mutable_image()->set_data_type("uint8");

    // expected FALSE as invalid message type
    EXPECT_FALSE(google::protobuf::util::MessageDifferencer::Equals(customReply, expectedReplyMsg));
}
