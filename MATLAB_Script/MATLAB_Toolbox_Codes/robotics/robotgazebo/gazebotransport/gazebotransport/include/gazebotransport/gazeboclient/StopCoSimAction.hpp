/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBO_STOP_CO_SIM_ACTION_HPP
#define GAZEBO_STOP_CO_SIM_ACTION_HPP

#include "gazebotransport/gazeboclient/Action.hpp"
#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h" // protobuf auto-generated header

#include <cstdint> // for uint32_t
#include <string>

namespace robotics {
namespace gazebotransport {
/// StopCoSimAction stop co-simulation with Gazebo simulator
/**
This action sends a StopCoSim request and expect a Packet with CoSimError::None
status reply.
*/
class StopCoSimAction : public Action {
  public:
    /// constructor
    StopCoSimAction();

    /// set co-simulation client id
    /**
    @param clientID                  unique id for co-simulation client
    */
    void setClientID(std::string const& clientID);

    /// create the serialized StopCoSim message
    /**
    @return                      serialized StopCoSim message
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
