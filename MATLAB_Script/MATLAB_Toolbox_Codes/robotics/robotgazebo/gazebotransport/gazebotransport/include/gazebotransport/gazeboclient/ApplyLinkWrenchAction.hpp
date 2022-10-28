/* Copyright 2019 The MathWorks, Inc. */
#ifndef APPLY_LINK_WRENCH_ACTION_HPP
#define APPLY_LINK_WRENCH_ACTION_HPP

#include "gazebotransport/gazeboclient/Action.hpp"
#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h" // protobuf auto-generated header

#include <cstdint> // for uint32_t
#include <string>

namespace robotics {
namespace gazebotransport {
/// ApplyLinkWrenchAction sends an Apply Link Wrench command to Gazebo server
class ApplyLinkWrenchAction : public Action {
  public:
    ApplyLinkWrenchAction();

    /// setup the link wrench command
    /**
    @param modelName				Gazebo model that owns the link
    @param linkName 				Gazebo link name
    @param forceType				Indicate whether the force is additive or not
    @param force					The amount of force to apply at link center
    of mass
    @param torqueType				Indicate whether the torque is additive or not
    @param torque					The amount of torque to apply at link center
    of mass
    @param durationSeconds			How long the torque should be applied: seconds +
    1e-9*nanoSeconds
    @param durationNanoSeconds		How long the torque should be applied: seconds +
    1e-9*nanoSeconds
    */
    void setLinkWrench(std::string const& modelName,
                       std::string const& linkName,
                       std::string const& forceType,
                       std::vector<double> const& force,
                       std::string const& torqueType,
                       std::vector<double> const& torque,
                       uint64_t durationSeconds,
                       uint64_t durationNanoSeconds);

    /// create the serialized ApplyLinkWrench message
    /**
    @return                      serialized ApplyLinkWrench message
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
    /// m_message Buffered ApplyLinkWrench command
    mw::internal::robotics::gazebotransport::Packet m_message;

    /// m_success Record whether last apply action is successful
    bool m_success;
};
} // namespace gazebotransport
} // namespace robotics

#endif
