/* Copyright 2019 The MathWorks, Inc. */
#include "mw_gtest/gtest.hpp"

#include "gazebotransport/Client.hpp"
#include "gazebotransport/GazeboServer.hpp"

#include "boost/date_time.hpp"

#include <iostream>
#include <limits>
#include <thread>
#include <chrono>

MWTEST(testGazeboServer, startAndStopConnection) {
    // check server's capability of start and stop connections from clients

    auto testServer = std::make_shared<robotics::gazebotransport::GazeboServer>(0);

    // try connect to server before it starts accepting should fail
    try {
        auto testClient = std::make_shared<robotics::gazebotransport::Client>(
            "127.0.0.1", testServer->getPortName(), boost::posix_time::milliseconds(100));
        FAIL();
    } catch (std::exception const& ex) {
        std::cout << "Captured exception: " << ex.what() << std::endl;
        SUCCEED();
    }

    // after server is running, client should be able to connect
    try {
        testServer->run();
        auto testClient = std::make_shared<robotics::gazebotransport::Client>(
            "127.0.0.1", testServer->getPortName(), boost::posix_time::milliseconds(100));
        SUCCEED();
    } catch (std::exception const& ex) {
        FAIL() << "Fail with exception: " << ex.what();
    }

    // server should be able to shutdown without error as well
    try {
        testServer->stop();
        SUCCEED();
    } catch (std::exception const& ex) {
        FAIL() << "Fail with exception: " << ex.what();
    }
}

MWTEST(testGazeboServer, startAndStopCoSimulation) {
    // check server's capability of starting and stopping co-simulation

    auto testServer = std::make_shared<robotics::gazebotransport::GazeboServer>(0);

    // check default co-simulation status
    auto status = testServer->getCoSimulationStatus();
    EXPECT_FALSE(status.first);
    EXPECT_EQ(status.second, "");

    // start co-simulation with test client id
    std::string testClientID = "TestClient";
    EXPECT_TRUE(
        testServer->startCoSimulation(testClientID, std::numeric_limits<double>::infinity()));
    status = testServer->getCoSimulationStatus();
    EXPECT_TRUE(status.first);
    EXPECT_EQ(status.second, testClientID);

    // start co-simulation with same test client id and inf duration will still keep connection
    EXPECT_TRUE(
        testServer->startCoSimulation(testClientID, std::numeric_limits<double>::infinity()));
    status = testServer->getCoSimulationStatus();
    EXPECT_TRUE(status.first);
    EXPECT_EQ(status.second, testClientID);

    // attempt to run co-simulation again with second client would fail
    std::string anotherClient = "OtherTestClient";
    EXPECT_FALSE(
        testServer->startCoSimulation(anotherClient, std::numeric_limits<double>::infinity()));
    status = testServer->getCoSimulationStatus();
    EXPECT_TRUE(status.first);
    EXPECT_EQ(status.second, testClientID);

    // attempt to stop co-simulation from second client would fail
    EXPECT_FALSE(testServer->stopCoSimulation(anotherClient));
    status = testServer->getCoSimulationStatus();
    EXPECT_TRUE(status.first);
    EXPECT_EQ(status.second, testClientID);

    // stop co-simulation with original client would pass
    EXPECT_TRUE(testServer->stopCoSimulation(testClientID));
    status = testServer->getCoSimulationStatus();
    EXPECT_FALSE(status.first);
    EXPECT_EQ(status.second, testClientID);

    // try to stop co-simulation while server is not running co-simulation
    // would pass for any client
    EXPECT_TRUE(testServer->stopCoSimulation(anotherClient));
    status = testServer->getCoSimulationStatus();
    EXPECT_FALSE(status.first);
    EXPECT_EQ(status.second, testClientID);
}

MWTEST(testGazeboServer, startCoSimulationWithTimeout) {
    // check server's capability of starting co-simulation and close it after time out
    auto testServer = std::make_shared<robotics::gazebotransport::GazeboServer>(0);

    // check default co-simulation status
    auto status = testServer->getCoSimulationStatus();
    EXPECT_FALSE(status.first);
    EXPECT_EQ(status.second, "");

    // start co-simulation with test client id for 1000 ms
    std::string testClientID = "TestClient";
    EXPECT_TRUE(testServer->startCoSimulation(testClientID, 1000));
    status = testServer->getCoSimulationStatus();
    EXPECT_TRUE(status.first);
    EXPECT_EQ(status.second, testClientID);

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // after 2000 ms, the co-simulation status should now be false
    status = testServer->getCoSimulationStatus();
    EXPECT_FALSE(status.first);
}

MWTEST(testGazeboServer, startCoSimulationWithExtension) {
    // check server's capability of starting co-simulation and close it after time out
    auto testServer = std::make_shared<robotics::gazebotransport::GazeboServer>(0);

    // check default co-simulation status
    auto status = testServer->getCoSimulationStatus();
    EXPECT_FALSE(status.first);
    EXPECT_EQ(status.second, "");

    // start co-simulation with test client id for 1000 ms
    std::string testClientID = "TestClient";
    EXPECT_TRUE(testServer->startCoSimulation(testClientID, 1000));
    status = testServer->getCoSimulationStatus();
    EXPECT_TRUE(status.first);
    EXPECT_EQ(status.second, testClientID);

    // after 500 ms, extend the co-simulation status by sending the startCoSimulation signal again
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    EXPECT_TRUE(testServer->startCoSimulation(testClientID, 1000));
    status = testServer->getCoSimulationStatus();
    EXPECT_TRUE(status.first);

    // after 500 ms, extend the co-simulation status by sending the startCoSimulation signal again
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    EXPECT_TRUE(testServer->startCoSimulation(testClientID, 1000));
    status = testServer->getCoSimulationStatus();
    EXPECT_TRUE(status.first);

    // after 500 ms, extend the co-simulation status by sending the startCoSimulation signal again
    // now we are passing the original 1000 ms timeout, but the server is still running
    // co-simulation
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    EXPECT_TRUE(testServer->startCoSimulation(testClientID, 1000));
    status = testServer->getCoSimulationStatus();
    EXPECT_TRUE(status.first);

    // extend with different client ID for 4000 ms, server would still exit co-simulation after 1000
    // ms
    EXPECT_FALSE(testServer->startCoSimulation("WrongID", 4000));
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    status = testServer->getCoSimulationStatus();
    EXPECT_FALSE(status.first);
}
