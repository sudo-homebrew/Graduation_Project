/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBOSTEPSIMULATIONACTION_HPP
#define GAZEBOSTEPSIMULATIONACTION_HPP

#include "gazebotransport/gazeboclient/Action.hpp"
#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h"

#include <cstdint> // for uint32_t
#include <string>

namespace robotics {
namespace gazebotransport {
/// StepSimulationAction steps the simulation in Gazebo server
/**
This action sends a StepSimulation request and expect a Packet with CoSimError::None status reply
*/
class StepSimulationAction : public Action {
  public:
    /// constructor
    StepSimulationAction();

    /// set step size of each step simulation action
    /**
    @param stepSize                  number of steps to use for each Step action
    */
    void setStepSize(uint32_t stepSize);

    /// create the serialized StepSimulation message
    /**
    @return                      serialized StepSimulation message
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
    @return                     indicates whether the last step action is successful
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
