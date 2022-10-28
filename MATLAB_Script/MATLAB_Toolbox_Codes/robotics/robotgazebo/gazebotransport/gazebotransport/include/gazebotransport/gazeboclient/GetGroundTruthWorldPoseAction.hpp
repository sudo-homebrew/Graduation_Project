/* Copyright 2019 The MathWorks, Inc. */
#ifndef GET_GROUND_TRUTH_POSE_ACTION_HPP
#define GET_GROUND_TRUTH_POSE_ACTION_HPP

#include "gazebotransport/gazeboclient/Action.hpp"
#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h"

#include "boost/optional.hpp"
#include <utility>
#include <memory>

namespace robotics {
namespace gazebotransport {
/// get ground truth pose for a model link in Gazebo world frame
class GetGroundTruthWorldPoseAction : public Action {
  public:
    GetGroundTruthWorldPoseAction();

    void setLinkName(std::string const& modelName, std::string const& linkName);

    void setSimulationTime(uint64_t seconds, uint64_t nanoSeconds);

    /// obtains the serialized message to send to the Gazebo server
    std::string getMsgToSend();

    /// process the optional returned serialized message from Gazebo server
    void processMsg(boost::optional<std::string> const& msg);

    /// get pose obtained in the last action
    std::pair<bool, std::shared_ptr<mw::internal::robotics::gazebotransport::Packet>> getLastPose()
        const;

  private:
    /// Indicates whether the message is update to date relative to simulation time
    bool m_isNew;

    /// Packet contains GetGroundTruthWorldPose message
    mw::internal::robotics::gazebotransport::Packet m_message;

    /// Pose as a protobuf message
    std::shared_ptr<mw::internal::robotics::gazebotransport::Packet> m_pose;
};
} // namespace gazebotransport
} // namespace robotics


#endif
