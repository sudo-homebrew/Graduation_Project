/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBO_REQUEST_CO_SIM_ACTION_HPP
#define GAZEBO_REQUEST_CO_SIM_ACTION_HPP

#include "gazebotransport/gazeboclient/Action.hpp"
#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h"

#include <cstdint> // for uint32_t
#include <string>

namespace robotics {
namespace gazebotransport {
/// RequestCoSimAction query whether Gazebo server is open for co-simulation request
/**
This action sends a RequestCoSim request and expect a Packet with CoSimError::None
status reply.
If Gazebo server is already running co-simulation with another client,
CoSimError:COSIM_FAILED would be received by client.
If the request time step is not a multiply of Gazebo solver basic time step,
CoSimError:COSIM_INCOMPATIBLE_TIMESTEP would be received by client.
*/
class RequestCoSimAction : public Action {
  public:
    /// constructor
    RequestCoSimAction();

    /// set client ID and co-simulation duration for the request co-simulation action
    /**
    @param clientID                  unique id for co-simulation client
    @param duration                  how long this co-simulation request should wait to check first
    heartbeat
    */
    void setRequest(std::string const& clientID, double duration);

    /// create the serialized RequestCoSim message
    /**
    @return                      serialized RequestCoSim message
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
    bool success() const;

  private:
    /// Packet contains StepSimulation message
    mw::internal::robotics::gazebotransport::Packet m_message;

    /// indicates whether the action is successful
    bool m_result;
};
} // namespace gazebotransport
} // namespace robotics

#endif
