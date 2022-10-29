/* Copyright 2020 The MathWorks, Inc. */
#ifndef SET_JOINT_VELOCITY_ACTION_HPP
#define SET_JOINT_VELOCITY_ACTION_HPP

#include "gazebotransport/gazeboclient/Action.hpp"
#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h" // protobuf auto-generated header

#include <cstdint> // for uint32_t
#include <string>

namespace robotics {
namespace gazebotransport {
/// SetJointVelocityAction sends an Set Joint Velocity command to Gazebo server
class SetJointVelocityAction : public Action {
  public:
    SetJointVelocityAction();

    /// setup joint velocity command
    /**
    @param modelName                Gazebo model that owns the joint
    @param jointName                Gazebo joint name
    @param indexValue               The axis of the joint to set velocity
    @param velocity                 The velocity value
    @param durationSeconds          How long the velocity should be set: seconds +
    1e-9*nanoSeconds
    @param durationNanoSeconds      How long the velocity should be set: seconds +
    1e-9*nanoSeconds
    */
    void setJointVelocity(std::string const& modelName,
                          std::string const& jointName,
                          uint32_t indexValue,
                          double velocity,
                          uint64_t durationSeconds,
                          uint64_t durationNanoSeconds);

    /// create the serialized SetJointVelocity message
    /**
    @return                      serialized SetJointVelocity message
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
    /// m_message Buffered SetJointVelocity command
    mw::internal::robotics::gazebotransport::Packet m_message;

    /// m_success Record whether last apply action is successful
    bool m_success;
};
} // namespace gazebotransport
} // namespace robotics

#endif
