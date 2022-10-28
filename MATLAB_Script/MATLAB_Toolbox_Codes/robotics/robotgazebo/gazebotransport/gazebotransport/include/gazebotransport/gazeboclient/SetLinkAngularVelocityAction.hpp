/* Copyright 2020 The MathWorks, Inc. */
#ifndef SET_LINK_ANGULAR_VELOCITY_ACTION_HPP
#define SET_LINK_ANGULAR_VELOCITY_ACTION_HPP

#include "gazebotransport/gazeboclient/Action.hpp"
#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h" // protobuf auto-generated header

#include <cstdint> // for uint32_t
#include <string>

namespace robotics {
namespace gazebotransport {
/// SetLinkAngularVelocityAction sends an Set Link Angular Velocity command to Gazebo server
class SetLinkAngularVelocityAction : public Action {
  public:
    SetLinkAngularVelocityAction();

    /// setup link angular velocity command
    /**
    @param modelName                Gazebo model that owns the link
    @param linkName                Gazebo link name
    @param velocity               	The velocity
        @param durationSeconds          How long the velocity should be set: seconds +
    1e-9*nanoSeconds
    @param durationNanoSeconds      How long the velocity should be set: seconds +
    1e-9*nanoSeconds
    */
    void setLinkAngularVelocity(std::string const& modelName,
                                std::string const& linkName,
                                std::vector<double> const& velocity,
                                uint64_t durationSeconds,
                                uint64_t durationNanoSeconds);

    /// create the serialized SetJointAngularVelocity message
    /**
    @return                      serialized SetJointAngularVelocity message
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
    /// m_message Buffered SetJointAngularVelocity command
    mw::internal::robotics::gazebotransport::Packet m_message;

    /// m_success Record whether last set action is successful
    bool m_success;
};
} // namespace gazebotransport
} // namespace robotics

#endif
