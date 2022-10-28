/* Copyright 2019 The MathWorks, Inc. */
#include "mw_gtest/gmock.hpp"

#include "gazebotransport/gazeboserver/RequestCoSimulationHandler.hpp"
#include "gazebotransport/GazeboServer.hpp"


using namespace robotics::gazebotransport;

/// Class to define Mock methods of Gazebo World
class MockGazeboWorld : public GazeboWorldInterface {
  public:
    /// Destructor
    ~MockGazeboWorld() {
    }

    /// MOCK method to set world pause state
    MOCK_METHOD1(setPaused, void(bool state));

    /// MOCK method to check simulation paused
    MOCK_METHOD0(isPaused, bool());

    /// MOCK method to step simulation
    MOCK_METHOD1(step, void(const uint32_t _steps));

    /// MOCK method to reset simulation scene and time
    MOCK_METHOD0(reset, void());

    /// MOCK method to reset simulation time
    MOCK_METHOD0(resetTime, void());
};


/// verify the reply string contains expected co-simulation error code
static void verifyPacketStatus(
    std::string const& msg,
    mw::internal::robotics::gazebotransport::Packet_CoSimError expectedStatus) {
    mw::internal::robotics::gazebotransport::Packet packet;
    EXPECT_TRUE(packet.ParseFromString(msg));
    EXPECT_TRUE(packet.has_status());
    EXPECT_EQ(packet.status(), expectedStatus);
}

class TestRequestCoSimHandler : public ::testing::Test {
  protected:
    void SetUp() {
        m_server = std::make_shared<GazeboServer>(0);
        m_world = std::make_shared<MockGazeboWorld>();
    }

    std::shared_ptr<GazeboServer> m_server;
    std::shared_ptr<MockGazeboWorld> m_world;
};

MWTEST_F(TestRequestCoSimHandler, constructor) {
    // check constructor and accepted ID
    RequestCoSimulationHandler handler{m_server, m_world};
    EXPECT_EQ(handler.getAcceptID(), mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                         PacketHeader_MsgID_REQUEST_COSIM);
}

MWTEST_F(TestRequestCoSimHandler, handleMessage) {
    RequestCoSimulationHandler handler{m_server, m_world};

    // For a wrong message, server would directly error
    mw::internal::robotics::gazebotransport::Packet msg;
    msg.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_STATUS);
    msg.mutable_header()->mutable_time_stamp()->set_seconds(0);
    msg.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    msg.set_status(
        mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE);
    auto reply = handler.handleMessage(msg);
    SCOPED_TRACE("Verify reply status");
    verifyPacketStatus(
        reply,
        mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_COSIM_FAILED);

    // process first request co-simulation message
    // This should put server into co-simulation stage
    std::string testClientID{"TestClient"};
    msg.mutable_header()->set_id(mw::internal::robotics::gazebotransport::PacketHeader_MsgID::
                                     PacketHeader_MsgID_REQUEST_COSIM);
    msg.mutable_header()->mutable_time_stamp()->set_seconds(0);
    msg.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    msg.mutable_request_cosim()->set_client_id(testClientID);
    msg.mutable_request_cosim()->set_duration(std::numeric_limits<double>::infinity());
    EXPECT_CALL(*m_world, setPaused(true)).Times(1);

    // For first request server would grant co-simulation
    reply = handler.handleMessage(msg);
    SCOPED_TRACE("Verify reply status");
    verifyPacketStatus(
        reply, mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_NONE);

    // Check that server is running co-simulation now
    EXPECT_TRUE(m_server->getCoSimulationStatus().first);
    EXPECT_EQ(m_server->getCoSimulationStatus().second, testClientID);

    // try to pass second request from another client would fail
    std::string otherClientID{"OtherTestClient"};
    msg.mutable_request_cosim()->set_client_id(otherClientID);

    // For second request server would reject co-simulation
    reply = handler.handleMessage(msg);
    SCOPED_TRACE("Verify reply status");
    verifyPacketStatus(
        reply,
        mw::internal::robotics::gazebotransport::Packet_CoSimError::Packet_CoSimError_COSIM_FAILED);
}
