/* Copyright 2020 The MathWorks, Inc. */
#include "gazebotransport/gazeboclient/checkErrorMessage.hpp"

std::string checkErrorMessage(boost::optional<std::string> const& packet) {
    if (packet) {
        mw::internal::robotics::gazebotransport::Packet inPacket;

        if (inPacket.ParseFromString(*packet) && inPacket.has_error_message()) {
            return inPacket.error_message();
        }
    }

    return "";
}
