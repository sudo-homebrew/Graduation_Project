/* Copyright 2020 The MathWorks, Inc. */
#ifndef GET_LINK_STATE_ACTION_HPP
#define GET_LINK_STATE_ACTION_HPP

#include "gazebotransport/gazeboclient/Action.hpp"
#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h"

#include "boost/optional.hpp"
#include <utility>
#include <memory>

namespace robotics {
namespace gazebotransport {
/// get link state for a model link in Gazebo world frame
class GetLinkStateAction : public Action {
  public:
    GetLinkStateAction();

    void setLinkName(std::string const& modelName, std::string const& linkName);

    void setSimulationTime(uint64_t seconds, uint64_t nanoSeconds);

    /// obtains the serialized message to send to the Gazebo server
    std::string getMsgToSend();

    /// process the optional returned serialized message from Gazebo server
    void processMsg(boost::optional<std::string> const& msg);

    /// get link state obtained in the last action
    std::pair<bool, std::shared_ptr<mw::internal::robotics::gazebotransport::Packet>>
    getLastLinkState() const;

  private:
    /// Indicates whether the message is update to date relative to simulation time
    bool m_isNew;

    /// Packet contains GetLinkState message
    mw::internal::robotics::gazebotransport::Packet m_message;

    /// Link State as a protobuf message
    std::shared_ptr<mw::internal::robotics::gazebotransport::Packet> m_linkState;
};
} // namespace gazebotransport
} // namespace robotics


#endif
