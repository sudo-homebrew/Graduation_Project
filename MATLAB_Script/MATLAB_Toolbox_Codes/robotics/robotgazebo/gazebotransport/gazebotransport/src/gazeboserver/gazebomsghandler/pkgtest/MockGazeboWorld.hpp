/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_MOCKGAZEBOWORLD_HPP
#define GAZEBOCOSIM_MOCKGAZEBOWORLD_HPP

#include "mw_gtest/gmock.hpp"
#include "gazebotransport/gazeboserver/GazeboWorldInterface.hpp" // Interface for the world pointer

using ::testing::AtLeast;

/// Class to define Mock methods of Gazebo World
class MockGazeboWorld : public robotics::gazebotransport::GazeboWorldInterface
{
      public:
        /// Destructor 
        ~MockGazeboWorld() {}
        
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
#endif
