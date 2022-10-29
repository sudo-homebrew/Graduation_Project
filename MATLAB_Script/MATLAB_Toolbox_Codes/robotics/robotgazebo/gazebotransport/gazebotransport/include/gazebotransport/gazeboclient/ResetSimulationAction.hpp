/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBO_RESET_SIMULATION_ACTION_HPP
#define GAZEBO_RESET_SIMULATION_ACTION_HPP

#include "gazebotransport/gazeboclient/Action.hpp"
#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h"

#include <cstdint> // for uint32_t
#include <string>

namespace robotics {
namespace gazebotransport {
/// ResetSimulationAction resets the simulation in Gazebo server
/**
This action sends a ResetSimulation request and expect a Packet with CoSimError::None status reply
*/
class ResetSimulationAction : public Action {
  public:
    /// constructor
    ResetSimulationAction();

    /// configure the reset behavior
    /**
    @param resetMode                  Different way to reset Gazebo
    */
    void setResetMode(
        mw::internal::robotics::gazebotransport::ResetSimulation_ResetBehavior resetMode);

    /// create the serialized ResetSimulation message
    /**
    @return                      serialized ResetSimulation message
    */
    std::string getMsgToSend() override;

    /// process the serialized server reply
    /**
    @param msg                   optional serialized server reply

    If msg is boost::none or cannot be deserialized into Packet with CoSimError::None status
    This action is not successful
    */
    void processMsg(boost::optional<std::string> const& msg) override;

    /// check whether last action is successful
    /**
    @return                     indicates whether the last reset action is successful
    */
    bool success() const;

  private:
    /// Packet contains StepSimulation message
    mw::internal::robotics::gazebotransport::Packet m_message;

    /// indicates whether the action is successful
    bool m_success;
};
} // namespace gazebotransport
} // namespace robotics

#endif
