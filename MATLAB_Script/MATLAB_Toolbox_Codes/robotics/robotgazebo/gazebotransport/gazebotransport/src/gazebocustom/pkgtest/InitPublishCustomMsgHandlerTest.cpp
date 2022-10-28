/* Copyright 2019 The MathWorks, Inc. */
#include "mw_gtest/gtest.hpp"

#include "gazebotransport/Client.hpp"

#include "gazebotransport/GazeboServer.hpp"
#include "TestCustomMsgHandler.hpp"
#include "gazebotransport/gazebocustom/CustomMsgDispatcher.hpp"
#include "gazebotransport/gazebocustom/gazebocustommsghandler/InitPublishCustomMsgHandler.hpp"

#include "gazebotransport/Server.hpp"
#include "gazebotransport/Callback.hpp"
#include "gazebotransport/PacketEnding.hpp"

/// Test InitPublishCustomMsgHandler functionality
/// These tests are performed to validate message type for InitPublishCustomMsgHandler handler
class InitPublishCustomMsgHandlerTest : public testing::Test {
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

        // register InitPublishCustomMsgHandler
        m_server->registerHandler(
            std::make_shared<robotics::gazebotransport::InitPublishCustomMsgHandler>(
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
/// Test InitPublishCustomMsgHandler for valid message type
MWTEST_F(InitPublishCustomMsgHandlerTest, InitPublishCustomMsgHandlerTestValidMsgType) {

    bool m_success = false;

    // create Packet message with INIT_CUSTOM_MESSAGE_PUBLISHER type
    mw::internal::robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID_INIT_CUSTOM_MESSAGE_PUBLISHER);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_init_custom_publisher()->set_topic_name("~/custom_message");
    m_message.mutable_init_custom_publisher()->set_message_type("gazebo_msgs/test_custom");

    // serialize message
    auto msg = m_message.SerializeAsString();

    // send packet message & receive reply message over server-client interaction
    auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(10000));

    if (replyMsg) {
        mw::internal::robotics::gazebotransport::Packet reply;
        reply.ParseFromString(*replyMsg);

        m_success = reply.has_status() && !reply.status();
    } else {
        m_success = false;
    }

    // expected TRUE as valid message type
    EXPECT_TRUE(m_success);
}

/// NEGATIVE TEST
/// Test InitPublishCustomMsgHandler for invalid message type
MWTEST_F(InitPublishCustomMsgHandlerTest, InitPublishCustomMsgHandlerTestInValidMsgType) {

    bool m_success = false;

    // create Packet message with INIT_CUSTOM_MESSAGE_PUBLISHER type
    mw::internal::robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID_INIT_CUSTOM_MESSAGE_PUBLISHER);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_init_custom_publisher()->set_topic_name("~/custom_message");
    m_message.mutable_init_custom_publisher()->set_message_type("gazebo_msgs/test_incorrect");

    // serialize message
    auto msg = m_message.SerializeAsString();

    // send packet message & receive reply message over server-client interaction
    auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(10000));

    if (replyMsg) {
        mw::internal::robotics::gazebotransport::Packet reply;
        reply.ParseFromString(*replyMsg);

        m_success = reply.has_status() && !reply.status();
    } else {
        m_success = false;
    }

    // expected FALSE as invalid message type
    EXPECT_FALSE(m_success);
}
