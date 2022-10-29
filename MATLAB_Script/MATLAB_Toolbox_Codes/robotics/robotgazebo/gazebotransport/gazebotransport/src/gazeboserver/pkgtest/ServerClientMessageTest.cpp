/* Copyright 2019 The MathWorks, Inc. */
#include "mw_gtest/gtest.hpp"

#include "gazebotransport/Client.hpp"

#include "gazebotransport/GazeboServer.hpp"
#include "ServerTestMsgHandler.hpp"

#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h"

#include "gazebotransport/Server.hpp"
#include "gazebotransport/Callback.hpp"
#include "gazebotransport/PacketEnding.hpp"

class ServerClientMessageTest : public testing::Test {
  public:
    std::shared_ptr<robotics::gazebotransport::GazeboServer> m_server;
    std::shared_ptr<robotics::gazebotransport::Client> m_client;

    std::string ipAddress = "127.0.0.1";

    void SetUp() {
        m_server = std::make_shared<robotics::gazebotransport::GazeboServer>(
            static_cast<uint16_t>(std::stoi("0")));
        m_server->run();
        m_client = std::make_shared<robotics::gazebotransport::Client>(
            ipAddress, m_server->getPortName(), boost::posix_time::milliseconds(10000));
    }

    void TearDown() {
        m_server->stop();
        m_client->shutdown();
    }

    void serverMsgHandleInit() {
        m_server->registerHandler(
            std::make_shared<robotics::gazebotransport::ServerTestMsgHandler>());
    }
};

MWTEST_F(ServerClientMessageTest, msgHandle1) {

    serverMsgHandleInit();

    bool m_success = false;

    mw::internal::robotics::gazebotransport::Packet m_message;

    m_message.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID_REQUEST_IMAGE);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_request_image()->set_topic_name("test_message");

    auto msg = m_message.SerializeAsString();

    auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(10000));

    if (replyMsg) {
        mw::internal::robotics::gazebotransport::Packet reply;
        reply.ParseFromString(*replyMsg);

        m_success = reply.has_status() && !reply.status();
    } else {
        m_success = false;
    }

    EXPECT_TRUE(m_success);
}
