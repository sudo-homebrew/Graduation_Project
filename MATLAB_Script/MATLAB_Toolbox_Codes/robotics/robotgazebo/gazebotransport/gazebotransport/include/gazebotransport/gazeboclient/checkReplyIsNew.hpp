#/* Copyright 2019 The MathWorks, Inc. */
#ifndef CHECK_ERROR_STATUS_HPP
#define CHECK_ERROR_STATUS_HPP

#include "mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h"

namespace robotics {
namespace gazebotransport {
/// check the reply message contains new information compared to before based on timestamp
inline bool checkReplyIsNew(mw::internal::robotics::gazebotransport::Packet const& lastReply,
                            mw::internal::robotics::gazebotransport::Packet const& reply) {
    return (lastReply.header().time_stamp().seconds() < reply.header().time_stamp().seconds()) ||
           (lastReply.header().time_stamp().seconds() == reply.header().time_stamp().seconds() &&
            lastReply.header().time_stamp().nano_seconds() <
                reply.header().time_stamp().nano_seconds());
}
} // namespace gazebotransport
} // namespace robotics


#endif
