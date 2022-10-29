/* Copyright 2020 The MathWorks, Inc. */
#ifndef CHECK_ERROR_MESSAGE_HPP
#define CHECK_ERROR_MESSAGE_HPP

#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h"
#include "boost/optional.hpp"

/// checkErrorMessage checks that whether packet contains error message or not
/**
@param packet                             Input protobuf packet
@return                                   Return error message if the packet contains a error
message payload. Return empty string "" otherwise.
*/
std::string checkErrorMessage(boost::optional<std::string> const& packet);


#endif
