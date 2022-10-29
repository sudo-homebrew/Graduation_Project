/* Copyright 2020 The MathWorks, Inc. */
#ifndef GET_GAZEBO_MODEL_PARAM_ACTION_HPP
#define GET_GAZEBO_MODEL_PARAM_ACTION_HPP

#include "gazebotransport/gazeboclient/Action.hpp"
#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h" // protobuf auto-generated header

#include <cstdint> // for uint32_t
#include <string>

namespace robotics {
namespace gazebotransport {
/// GetGazeboModelParamAction sends an Get Gazebo Model param command to Gazebo server
class GetGazeboModelParamAction : public Action {
  public:
    GetGazeboModelParamAction();

    /// setup the get gazebo model param command
    /**
    @param modelName                Gazebo model name
    @param isLink                   Requested is link or joint ( true or false )
    @param linkJointName            Gazebo link or joint name
    */
    void setModelName(std::string const& modelName, bool isLink, std::string const& linkJointName);

    void setSimulationTime(uint64_t seconds, uint64_t nanoSeconds);

    /// get Gazebo model param info obtained in the last action
    std::pair<bool, std::shared_ptr<mw::internal::robotics::gazebotransport::Packet>>
    getGazeboModelParam() const;

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

    /// Packet contains GetGazeboModelParam message
    mw::internal::robotics::gazebotransport::Packet m_message;

    /// Gazebo model param as a protobuf message
    std::shared_ptr<mw::internal::robotics::gazebotransport::Packet> m_gazeboModelParam;
};
} // namespace gazebotransport
} // namespace robotics

#endif
