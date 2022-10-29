/* Copyright 2019 The MathWorks, Inc. */
#ifndef TESTCUSTOMMSGHANDLER_HPP
#define TESTCUSTOMMSGHANDLER_HPP

#include <memory>
#include <cstdint>
#include <string>
#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h"
#include "gazebotransport/gazebocustom/CustomMsgHandler.hpp"

using namespace mw::internal::robotics::gazebotransport;

namespace robotics {
namespace gazebotransport {
/// This class is used to test custom message handler functionality
/// It includes dummy functions, which are created in the gazebogenmsg()
/// for custom message support.

class TestCustomMsgHandler : public CustomMsgHandler {
  public:
    /// Constructor
    /// This class initiated test custom message handler
    explicit TestCustomMsgHandler();

    /// Destructor
    ~TestCustomMsgHandler();

    /**
    @param msgContent      Packet message

    Handles Packet message & provide serialized string of error message
        based on initialization of custom message publisher
    */
    std::string initPublisher(Packet const& msgContent);

    /**
    @param msgContent      Packet message

    Handles Packet message & provide serialized string of error message
        based on initialization of custom message subscriber
    */
    std::string initSubscriber(Packet const& msgContent);

    /**
    @param msgContent      Packet message

    Handles Packet message & provide serialized string of error message
        based on published custom message
    */
    std::string publishCustomMsg(Packet const& msgContent);

    /**
    @param msgContent      Packet message

    Handles Packet message & provide serialized string of custom message
    */
    std::pair<bool, std::string> subscribeCustomMsg(Packet const& msgContent);

    // reset internal storage
    void resetInternal();
};

} // namespace gazebotransport
} // namespace robotics

#endif
