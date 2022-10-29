/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBO_GET_TOPIC_LIST_ACTION_HPP
#define GAZEBO_GET_TOPIC_LIST_ACTION_HPP

#include "gazebotransport/gazeboclient/Action.hpp"
#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h"

#include <cstdint> // for uint32_t
#include <string>

namespace robotics {
namespace gazebotransport {
/// GetTopicListAction query Gazebo topic list
/**
This action sends a GetTopicList request and expect a TopicList packet reply
*/
class GetTopicListAction : public Action {
  public:
    /// constructor
    GetTopicListAction();

    /// create the serialized GetTopicList message
    /**
    @return                      serialized GetTopicList message
    */
    std::string getMsgToSend() override;

    /// process the serialized server reply
    /**
    @param msg                   optional serialized server reply

    If msg is boost::none or cannot be deserialized into TopicList Packet,
    This action is not successful
    */
    void processMsg(boost::optional<std::string> const& msg) override;

    /// return the topic list obtained from Gazebo
    std::shared_ptr<mw::internal::robotics::gazebotransport::Packet> getTopicList();

  private:
    /// Packet contains GetTopicList message
    mw::internal::robotics::gazebotransport::Packet m_message;

    /// Contains the last received topic list
    std::shared_ptr<mw::internal::robotics::gazebotransport::Packet> m_buffer;
};
} // namespace gazebotransport
} // namespace robotics

#endif
