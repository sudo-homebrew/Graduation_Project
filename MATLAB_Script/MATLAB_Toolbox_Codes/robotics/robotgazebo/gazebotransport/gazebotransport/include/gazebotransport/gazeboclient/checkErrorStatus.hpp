/* Copyright 2019 The MathWorks, Inc. */
#ifndef CHECK_ERROR_STATUS_HPP
#define CHECK_ERROR_STATUS_HPP

#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h"
#include "boost/optional.hpp"

/// checkErrorStatus checks that whether packet contains error status or not
/**
@param packet                             Input protobuf packet
@return                                   Return status if the packet contains a status payload.
                                          Return boost::none otherwise.
*/
boost::optional<mw::internal::robotics::gazebotransport::Packet_CoSimError> checkErrorStatus(
    boost::optional<std::string> const& packet);


#endif
