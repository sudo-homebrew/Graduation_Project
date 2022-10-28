/* Copyright 2019 The MathWorks, Inc. */
#include "MockGazeboWorld.hpp"
#include "gazebotransport/gazeboserver/gazebomsghandler/StepSimulationMsgHandler.hpp"
#include "gazebotransport/GazeboServer.hpp"
#include "gazebotransport/Client.hpp"
#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h"

using testing::Return;

/// Reset Simulation message test for gtest
class StepSimulationMessageTest : public testing::Test {
  public:
    std::shared_ptr<robotics::gazebotransport::GazeboServer> m_server;
    std::shared_ptr<robotics::gazebotransport::Client> m_client;
    std::shared_ptr<MockGazeboWorld> m_mockWorld;
    std::string ipAddress = "127.0.0.1";

    void SetUp() {
        /// Launch GazeboServer
        m_server = std::make_shared<robotics::gazebotransport::GazeboServer>(
            static_cast<uint16_t>(std::stoi("0")));
        m_server->run();

        /// Launch Client
        m_client = std::make_shared<robotics::gazebotransport::Client>(
            ipAddress, m_server->getPortName(), boost::posix_time::milliseconds(10000));

        /// Create mock Gazebo World
        m_mockWorld = std::make_shared<MockGazeboWorld>();

        /// Launch Step Simulation message handler
        m_server->registerHandler(
            std::make_shared<robotics::gazebotransport::StepSimulationMsgHandler>(m_mockWorld));
    }

    void TearDown() {
        m_server->stop();
        m_client->shutdown();
    }
};

/// Test gazebo simulation stepping
MWTEST_F(StepSimulationMessageTest, stepping) {
    uint32_t ss = 1;

    EXPECT_CALL(*m_mockWorld, isPaused()).WillOnce(Return(true));
    EXPECT_CALL(*m_mockWorld, step(ss)).Times(1);

    mw::internal::robotics::gazebotransport::Packet m_message;

    m_message.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID_STEP_SIMULATION);
    m_message.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_message.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_message.mutable_step_simulation()->set_num_steps(1);

    auto msg = m_message.SerializeAsString();
    auto replyMsg = m_client->write(msg, boost::posix_time::milliseconds(10000));
    bool m_success;

    mw::internal::robotics::gazebotransport::Packet reply;

    if (replyMsg) {
        reply.ParseFromString(*replyMsg);

        m_success = reply.has_status() && !reply.status();
    } else {
        m_success = false;
    }

    ASSERT_TRUE(m_success);
}
