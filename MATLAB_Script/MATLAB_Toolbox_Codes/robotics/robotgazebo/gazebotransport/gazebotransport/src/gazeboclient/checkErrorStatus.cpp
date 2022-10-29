/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/gazeboclient/checkErrorStatus.hpp"

boost::optional<mw::internal::robotics::gazebotransport::Packet_CoSimError> checkErrorStatus(
    boost::optional<std::string> const& packet) {
    if (packet) {
        mw::internal::robotics::gazebotransport::Packet inPacket;

        if (inPacket.ParseFromString(*packet) && inPacket.has_status()) {
            return inPacket.status();
        }
    }

    return boost::none;
}
