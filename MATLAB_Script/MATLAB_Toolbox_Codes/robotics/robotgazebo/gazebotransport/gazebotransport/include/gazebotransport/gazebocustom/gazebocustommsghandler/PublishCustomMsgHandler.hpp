/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBOCOSIM_PUBLISHCUSTOMMSGHANDLER_HPP
#define GAZEBOCOSIM_PUBLISHCUSTOMMSGHANDLER_HPP

#include "gazebotransport/gazeboserver/MsgHandler.hpp"
#include "gazebotransport/gazebocustom/CustomMsgDispatcher.hpp"

namespace robotics {
namespace gazebotransport {
class PublishCustomMsgHandler : public MsgHandler {
  public:
    /// Constructor
    /// This class initiates message handler to exchange Publish custom message
    /*
    @param customDispatcher             CustomMsgDispatcher object
    */
    explicit PublishCustomMsgHandler(std::shared_ptr<CustomMsgDispatcher> customDispatcher);

    /// Destructor
    ~PublishCustomMsgHandler() override;

    /**
    @param msgContent      Packet message

    Handles Packet message & construct new Packet message with proper fields
    (Error/ reply content). Based on input message, publishes custom message on Gazebo.
    Further, serializes Packet ( reply message) into string and returns.
    */
    std::string handleMessage(
        mw::internal::robotics::gazebotransport::Packet const& msgContent) override;

    /// Returns Message ID
    uint32_t getAcceptID() const override;

  private:
    /// Custom Message Dispatcher pointer
    std::shared_ptr<CustomMsgDispatcher> m_customDispatcher;
};
} // namespace gazebotransport
} // namespace robotics
#endif
