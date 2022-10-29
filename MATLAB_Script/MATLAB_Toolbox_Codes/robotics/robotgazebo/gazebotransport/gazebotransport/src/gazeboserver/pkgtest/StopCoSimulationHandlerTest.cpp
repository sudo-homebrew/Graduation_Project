/* Copyright 2019 The MathWorks, Inc. */
#include "mw_gtest/gtest.hpp"

#include "gazebotransport/gazeboserver/StopCoSimulationHandler.hpp"
#include "gazebotransport/GazeboServer.hpp"

#include <limits>

using namespace robotics::gazebotransport;

/// verify the reply string contains expected co-simulation error code
static void verifyPacketStatus(
    std::string const& msg,
    mw::internal::robotics::gazebotransport::Packet_CoSimError expectedStatus) {
    mw::internal::robotics::gazebotransport::Packet packet;
    EXPECT_TRUE(packet.ParseFromString(msg));
    EXPECT_TRUE(packet.has_status());
    EXPECT_EQ(packet.status(), expectedStatus);
}

class TestStopCoSimulationHandler : public ::testing::Test {
  protected:
    void SetUp() {
        m_server = std::make_shared<GazeboServer>(0);
    }

    std::shared_ptr<GazeboServer> m_server;
};

MWTEST_F(TestStopCoSimulationHandler, constructor) {
    // check constructor and accepted ID
    StopCoSimulationHandler handler{m_server};
    EXPECT_EQ(
        handler.getAcceptID(),
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_STOP_COSIM);
}

MWTEST_F(TestStopCoSimulationHandler, handleMessage) {
    StopCoSimulationHandler handler{m_server};

    // For a wrong message, server would directly reject
    mw::internal::robotics::gazebotransport::Packet msg;
    msg.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_STATUS);
    msg.mutable_header()->mutable_time_stamp()->set_seconds(0);
    msg.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    msg.set_status(
        mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE);
    auto reply = handler.handleMessage(msg);
    SCOPED_TRACE("Verify reply status");
    verifyPacketStatus(reply, mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                  Packet_CoSimError_STOP_COSIM_FAILED);

    // process first stop co-simulation message before server is running
    // Server would return no-error to the request
    std::string testClientID{"TestClient"};
    msg.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_STOP_COSIM);
    msg.mutable_header()->mutable_time_stamp()->set_seconds(0);
    msg.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    msg.mutable_stop_cosim()->set_client_id(testClientID);

    // For first request server would return no-error
    reply = handler.handleMessage(msg);
    SCOPED_TRACE("Verify reply status");
    verifyPacketStatus(
        reply, mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE);

    // make server into running state for 1 second
    m_server->startCoSimulation(testClientID, std::numeric_limits<double>::infinity());

    // try to pass stop request from another client would fail
    std::string otherClientID{"OtherTestClient"};
    msg.mutable_stop_cosim()->set_client_id(otherClientID);

    // For request from different client, server would reject stop co-simulation call
    reply = handler.handleMessage(msg);
    SCOPED_TRACE("Verify reply status");
    verifyPacketStatus(reply, mw::internal::robotics::gazebotransport::Packet_CoSimError::
                                  Packet_CoSimError_STOP_COSIM_FAILED);

    // Check that server is still running co-simulation
    EXPECT_TRUE(m_server->getCoSimulationStatus().first);
    EXPECT_EQ(m_server->getCoSimulationStatus().second, testClientID);

    // Now send stop request from the original client
    msg.mutable_stop_cosim()->set_client_id(testClientID);

    // server should accept and return no-error
    reply = handler.handleMessage(msg);
    SCOPED_TRACE("Verify reply status");
    verifyPacketStatus(
        reply, mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE);

    // Check that server stops running co-simulation
    EXPECT_FALSE(m_server->getCoSimulationStatus().first);
}
