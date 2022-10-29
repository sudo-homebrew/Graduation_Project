/* Copyright 2019 The MathWorks, Inc. */
#ifndef APPLY_JOINT_TORQUE_ACTION_HPP
#define APPLY_JOINT_TORQUE_ACTION_HPP

#include "gazebotransport/gazeboclient/Action.hpp"
#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h" // protobuf auto-generated header

#include <cstdint> // for uint32_t
#include <string>

namespace robotics {
namespace gazebotransport {
/// ApplyJointTorqueAction sends an Apply Joint Torque command to Gazebo server
class ApplyJointTorqueAction : public Action {
  public:
    ApplyJointTorqueAction();

    /// setup the torque command
    /**
    @param modelName                Gazebo model that owns the joint
    @param jointName                Gazebo joint name
    @param indexValue               The axis of the joint to apply torque to
    @param effortValue              The force/torque amount
    @param durationSeconds          How long the torque should be applied: seconds +
    1e-9*nanoSeconds
    @param durationNanoSeconds      How long the torque should be applied: seconds +
    1e-9*nanoSeconds
    */
    void setJointTorque(std::string const& modelName,
                        std::string const& jointName,
                        uint32_t indexValue,
                        double effortValue,
                        uint64_t durationSeconds,
                        uint64_t durationNanoSeconds);

    /// create the serialized ApplyJointTorque message
    /**
    @return                      serialized ApplyJointTorque message
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
    /// m_message Buffered ApplyJointTorque command
    mw::internal::robotics::gazebotransport::Packet m_message;

    /// m_success Record whether last apply action is successful
    bool m_success;
};
} // namespace gazebotransport
} // namespace robotics

#endif
