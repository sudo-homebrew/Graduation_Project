/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/gazeboclient/GazeboClientImpl.hpp"
#include "gazebotransport/gazeboclient/Action.hpp"
#include "gazebotransport/Client.hpp"

#include "boost/date_time.hpp"

namespace robotics {
namespace gazebotransport {
GazeboClientImpl::GazeboClientImpl(std::string const& ipaddress, uint16_t port, uint64_t timeout)
    : m_client(
          std::make_shared<Client>(ipaddress,
                                   std::to_string(port),
                                   boost::posix_time::milliseconds(static_cast<int64_t>(timeout))))
    , m_timeout(timeout) {
}

void GazeboClientImpl::act(Action& actor) {
    // Perform the action by send the request message and process server reply
    auto msg = actor.getMsgToSend();
    auto reply =
        m_client->write(msg, boost::posix_time::milliseconds(static_cast<int64_t>(m_timeout)));
    actor.processMsg(reply);
}

void GazeboClientImpl::shutdown() {
    m_client->shutdown();
}

} // namespace gazebotransport

} // namespace robotics
