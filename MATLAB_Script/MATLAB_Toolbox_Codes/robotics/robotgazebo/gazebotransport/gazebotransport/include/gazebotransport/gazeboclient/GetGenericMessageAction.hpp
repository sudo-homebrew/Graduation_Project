/* Copyright 2019 The MathWorks, Inc. */
#ifndef GET_GENERIC_MESSAGE_ACTION_HPP
#define GET_GENERIC_MESSAGE_ACTION_HPP

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

/// GetGenericMessageAction an action to retrieve data from Gazebo server based on topic names
/**
This action sends a RequestMessage request and expect a Image reply
*/
class GetGenericMessageAction : public Action {
  public:
    /// constructor
    GetGenericMessageAction();

    /// set the topic that this action would retrieve from
    /**
                @param topicType             topic type to retrieve from Gazebo
    @param topicName             topic name in Gazebo

                Currently support Image, FusedIMU and LidarScan as topic type
    */
    void setTopic(std::string const& topicType, std::string const& topicName);

    /// set the timestamp in the request image message
    void setSimulationTime(uint64_t seconds, uint64_t nanoSeconds);

    /// create the serialized RequestImage message
    /**
    @return                      serialized RequestImage message
    */
    std::string getMsgToSend() override;

    /// process the serialized server reply
    /**
    @param msg                   optional serialized server reply

    If msg is boost::none or cannot be deserialized into Image message
    This action is not successful
    */
    void processMsg(boost::optional<std::string> const& msg) override;

    /// get message from the last action taken by client
    /**
    @return                      return a pair indicating whether the message is new and a pointer
    to message data

    If last action is not successful, message pointer would be nullptr
    */
    std::pair<bool, std::shared_ptr<mw::internal::robotics::gazebotransport::Packet>> getMessage()
        const;

  private:
    /// Packet contains the Request message
    mw::internal::robotics::gazebotransport::Packet m_request;

    /// Topic type this action requested
    std::string m_requestType;

    /// Map from topic type to request generators
    std::unordered_map<std::string, std::unique_ptr<GenericMessageHandler>> m_generators;

    /// Message buffer stored in the action
    std::shared_ptr<mw::internal::robotics::gazebotransport::Packet> m_buffer;

    /// indicates whether the action is successful
    bool m_isNew;
};
} // namespace gazebotransport
} // namespace robotics


#endif
