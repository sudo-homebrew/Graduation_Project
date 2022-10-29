/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBOACTION_HPP
#define GAZEBOACTION_HPP

#include <string>

#include "boost/optional.hpp"

namespace robotics {
namespace gazebotransport {
/// Action interface for implementing each Gazebo client action
/**
Each Gazebo client action involves two steps:
Send a serialized co-simulation message to the Gazebo server
Process the serialized co-simulation message replied by the server

Both steps may have side-effects on the action class itself
*/
class Action {
  public:
    /// destructor
    virtual ~Action() = default;

    /// obtains the serialized message to send to the Gazebo server
    virtual std::string getMsgToSend() = 0;

    /// process the optional returned serialized message from Gazebo server
    virtual void processMsg(boost::optional<std::string> const& msg) = 0;
};
} // namespace gazebotransport
} // namespace robotics

#endif
