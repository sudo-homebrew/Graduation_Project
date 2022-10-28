/* Copyright 2019 The MathWorks, Inc. */
#include "mw_gtest/gtest.hpp"

#include "TestCustomMsgHandler.hpp"
#include "gazebotransport/gazebocustom/CustomMsgDispatcher.hpp"
#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h"

/// Test CustomMsgHandler & CustomMsgDispatcher functionality for custom message support.
/// The customMsgHandler stores the registered custom message handler.
/// Thus, CustomHandlerTest validates the input message type for different functionalities.

class CustomHandlerTest : public testing::Test {
  public:
    /// create CustomMsgDispatcher object
    std::shared_ptr<robotics::gazebotransport::CustomMsgDispatcher> m_customDispatch;

    void SetUp() {

        this->m_customDispatch = std::make_shared<robotics::gazebotransport::CustomMsgDispatcher>();

        // register test custom message handler
        this->m_customDispatch->registerCustomHandler(
            std::make_shared<robotics::gazebotransport::TestCustomMsgHandler>(),
            "gazebo_msgs/test_custom");
    }

    void TearDown() {
        // reset handler
        this->m_customDispatch->reset();
    }
};

/// NEGATIVE TEST
/// Test customMsgHandler for init custom message publisher  call with incorrect/invalid message
/// type
MWTEST_F(CustomHandlerTest, initPublisherTestInCorrectMsgType) {

    // create packet message with init custom message publisher
    mw::internal::robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID_INIT_CUSTOM_MESSAGE_PUBLISHER);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_init_custom_publisher()->set_topic_name("test_topic");
    m_message.mutable_init_custom_publisher()->set_message_type("gazebo_msgs/test_incorrect");

    // call initPublisher() of customMsgHandler
    std::string replyString = this->m_customDispatch->initPublisher(
        m_message.init_custom_publisher().message_type(), m_message);

    // expected false as invalid message type
    ASSERT_STREQ(replyString.c_str(), "");
}

/// POSITIVE TEST
/// Test customMsgHandler for init custom message publisher  call with valid message type
MWTEST_F(CustomHandlerTest, initPublisherTestCorrectMsgType) {

    // create packet message with init custom message publisher
    mw::internal::robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID_INIT_CUSTOM_MESSAGE_PUBLISHER);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_init_custom_publisher()->set_topic_name("test_topic");
    m_message.mutable_init_custom_publisher()->set_message_type("gazebo_msgs/test_custom");

    // call initPublisher() of customMsgHandler
    std::string replyString = this->m_customDispatch->initPublisher(
        m_message.init_custom_publisher().message_type(), m_message);

    // expected true as valid message type
    ASSERT_STRNE(replyString.c_str(), "");
}

/// NEGATIVE TEST
/// Test customMsgHandler for init custom message subscriber  call with invalid message type
MWTEST_F(CustomHandlerTest, initSubscriberTestInCorrectMsgType) {

    // create packet message with init custom message subscriber
    mw::internal::robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID_INIT_CUSTOM_MESSAGE_SUBSCRIBER);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_init_custom_subscriber()->set_topic_name("test_topic");
    m_message.mutable_init_custom_subscriber()->set_message_type("gazebo_msgs/test_incorrect");

    // call initSubscriber() of customMsgHandler
    std::string replyString = this->m_customDispatch->initSubscriber(
        m_message.init_custom_subscriber().message_type(), m_message);

    // expected false as invalid message type
    ASSERT_STREQ(replyString.c_str(), "");
}

/// POSITIVE TEST
/// Test customMsgHandler for init custom message subscriber  call with valid message type
MWTEST_F(CustomHandlerTest, initSubscriberTestCorrectMsgType) {

    // create packet message with init custom message subscriber
    mw::internal::robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID_CUSTOM_MESSAGE_PUBLISHER);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_init_custom_subscriber()->set_topic_name("test_topic");
    m_message.mutable_init_custom_subscriber()->set_message_type("gazebo_msgs/test_custom");

    // call initSubscriber() of customMsgHandler
    std::string replyString = this->m_customDispatch->initSubscriber(
        m_message.init_custom_subscriber().message_type(), m_message);

    // expected true as valid message type
    ASSERT_STRNE(replyString.c_str(), "");
}

/// NEGATIVE TEST
/// Test customMsgHandler for publish custom message call with invalid message type
MWTEST_F(CustomHandlerTest, publishTestInCorrectMsgType) {

    // create packet message with publish custom message
    mw::internal::robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID_CUSTOM_MESSAGE_PUBLISHER);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_custom_message_support()->set_topic_name("test_topic");
    m_message.mutable_custom_message_support()->set_message_type("gazebo_msgs/test_incorrect");
    m_message.mutable_custom_message_support()->set_data("test_custom_message_data");

    // call publishCustomMsg() of customMsgHandler
    std::string replyString = this->m_customDispatch->publishCustomMsg(
        m_message.custom_message_support().message_type(), m_message);

    // expected false as invalid message type
    ASSERT_STREQ(replyString.c_str(), "");
}

/// POSITIVE TEST
/// Test customMsgHandler for publish custom message call with valid message type
MWTEST_F(CustomHandlerTest, publishTestCorrectMsgType) {

    // create packet message with publish custom message
    mw::internal::robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID_CUSTOM_MESSAGE_PUBLISHER);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_custom_message_support()->set_topic_name("test_topic");
    m_message.mutable_custom_message_support()->set_message_type("gazebo_msgs/test_custom");
    m_message.mutable_custom_message_support()->set_data("test_custom_message_data");

    // call publishCustomMsg() of customMsgHandler
    std::string replyString = this->m_customDispatch->publishCustomMsg(
        m_message.custom_message_support().message_type(), m_message);

    // expected no string as valid message type
    ASSERT_STRNE(replyString.c_str(), "");
}

/// NEGATIVE TEST
/// Test customMsgHandler for subscribe custom message call with invalid message type
MWTEST_F(CustomHandlerTest, subscribeTestInCorrectMsgType) {

    // create packet message with subscribe custom message
    mw::internal::robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID_CUSTOM_MESSAGE_SUBSCRIBER);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_request_custom_message_support()->set_topic_name("test_topic");
    m_message.mutable_request_custom_message_support()->set_message_type(
        "gazebo_msgs/test_incorrect");

    // call subscribeCustomMsg() of customMsgHandler
    auto replyString = this->m_customDispatch->subscribeCustomMsg(
        m_message.request_custom_message_support().message_type(), m_message);

    // expected no string as invalid message type
    ASSERT_STREQ(replyString.second.c_str(), "");
}

/// POSITIVE TEST
/// Test customMsgHandler for subscribe custom message call with valid message type
MWTEST_F(CustomHandlerTest, subscribeTestCorrectMsgType) {

    // create packet message with subscribe custom message
    mw::internal::robotics::gazebotransport::Packet m_message;
    m_message.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID_CUSTOM_MESSAGE_SUBSCRIBER);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_request_custom_message_support()->set_topic_name("test_topic");
    m_message.mutable_request_custom_message_support()->set_message_type("gazebo_msgs/test_custom");

    // call subscribeCustomMsg() of customMsgHandler
    auto replyString = this->m_customDispatch->subscribeCustomMsg(
        m_message.request_custom_message_support().message_type(), m_message);

    // expected a serialized string as valid message type
    ASSERT_STRNE(replyString.second.c_str(), "");
}
