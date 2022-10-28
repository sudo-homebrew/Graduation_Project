/* Copyright 2019 The MathWorks, Inc. */
#ifndef SERVERTESTMSGHANDLER_HPP
#define SERVERTESTMSGHANDLER_HPP

#include "gazebotransport/gazeboserver/MsgHandler.hpp"
#include <map>
#include "mutex"

namespace robotics {
namespace gazebotransport {
class ServerTestMsgHandler : public MsgHandler {
  public:
    /// Constructor
    /// This class initiated message handler to test server message exchange
    explicit ServerTestMsgHandler();

    /**
    @param msgContent      Packet message

    Handles Packet message & construct new Packet message with proper fields (Error/ reply content
    ). Further, serializes Packet message into string and returns.
    */
    std::string handleMessage(
        mw::internal::robotics::gazebotransport::Packet const& msgContent) override;

    /// Returns Message ID
    uint32_t getAcceptID() const override;
};
} // namespace gazebotransport
} // namespace robotics
#endif
