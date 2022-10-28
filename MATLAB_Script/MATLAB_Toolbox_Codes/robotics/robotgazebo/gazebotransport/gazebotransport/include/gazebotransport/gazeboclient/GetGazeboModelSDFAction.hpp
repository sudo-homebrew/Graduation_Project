/* Copyright 2021 The MathWorks, Inc. */
#ifndef GET_GAZEBO_MODEL_SDF_ACTION_HPP
#define GET_GAZEBO_MODEL_SDF_ACTION_HPP

#include "gazebotransport/gazeboclient/Action.hpp"
#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h" // protobuf auto-generated header

#include <cstdint> // for uint32_t
#include <string>

namespace robotics {
namespace gazebotransport {
/// GetGazeboModelSDFAction sends an Get Gazebo Model SDF command to Gazebo server
class GetGazeboModelSDFAction : public Action {
  public:
    GetGazeboModelSDFAction();

    /// setup the get gazebo model SDF command
    /**
    @param modelName                Gazebo model name
    */
    void setModelName(std::string const& modelName);

    void setSimulationTime(uint64_t seconds, uint64_t nanoSeconds);

    /// get Gazebo model SDF details obtained in the last action
    std::pair<bool, std::shared_ptr<mw::internal::robotics::gazebotransport::Packet>>
    getGazeboModelSDF() const;

    /// create the serialized Gazebo model message
    /**
    @return                      serialized Gazebo model message
    */
    std::string getMsgToSend() override;

    /// process the serialized server reply
    /**
    @param msg                   optional serialized server reply

    If msg is boost::none or cannot be deserialized into Packet with CoSimError::None status
    This action is not successful
    */
    void processMsg(boost::optional<std::string> const& msg) override;


  private:
    /// Indicates whether the message is update
    bool m_isNew;

    /// Packet contains GetGazeboModelSDF message
    mw::internal::robotics::gazebotransport::Packet m_message;

    /// Gazebo model SDF as a protobuf message
    std::shared_ptr<mw::internal::robotics::gazebotransport::Packet> m_gazeboModelSDF;
};
} // namespace gazebotransport
} // namespace robotics

#endif
