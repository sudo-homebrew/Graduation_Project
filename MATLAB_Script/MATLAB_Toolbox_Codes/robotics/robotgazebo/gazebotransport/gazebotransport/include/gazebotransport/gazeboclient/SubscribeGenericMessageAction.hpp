/* Copyright 2019 The MathWorks, Inc. */
#ifndef SUBSCRIBE_IMAGE_ACTION_HPP
#define SUBSCRIBE_IMAGE_ACTION_HPP

#include "gazebotransport/gazeboclient/Action.hpp"
#include "gazebotransport/gazeboclient/GenericMessageHandlers.hpp"

#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h"

#include "boost/optional.hpp"

#include <cstdint> // for uint32_t
#include <string>
#include <utility>
#include <unordered_map>
#include <memory>

namespace robotics {
namespace gazebotransport {
/// SubscribeGenericMessageAction an action to subscribe topic message from Gazebo server
/**
This action sends a Subscribe* request and expect a status reply
*/
class SubscribeGenericMessageAction : public Action {
  public:
    /// constructor
    SubscribeGenericMessageAction();

    /// set the topic that this action would subscribe to
    /**
    @param topicType             type of topic to subscribe
    @param topicName             topic name of the message in Gazebo
    @return whether topic type is compatible
    */
    bool setTopic(std::string const& topicType, std::string const& topicName);

    /// create the serialized Subscribe* message
    /**
    @return                      serialized Subscribe* message
    */
    std::string getMsgToSend() override;

    /// process the serialized server reply
    /**
    @param msg                   optional serialized server reply

    If msg is boost::none or cannot be deserialized into Status message
    This action is not successful
    */
    void processMsg(boost::optional<std::string> const& msg) override;

    /// return whether the last subscription is successful
    bool success() const;

  private:
    /// Packet contains Subscribe* message
    mw::internal::robotics::gazebotransport::Packet m_message;

    /// Map from topic type to subscribe message generators
    std::unordered_map<std::string, std::unique_ptr<GenericMessageHandler>> m_generators;

    /// indicates whether the action is successful
    bool m_success;
};
} // namespace gazebotransport
} // namespace robotics


#endif
