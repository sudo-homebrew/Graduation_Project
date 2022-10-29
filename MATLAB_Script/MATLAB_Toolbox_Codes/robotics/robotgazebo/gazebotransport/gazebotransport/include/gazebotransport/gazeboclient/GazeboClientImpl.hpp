/* Copyright 2019 The MathWorks, Inc. */
#ifndef GAZEBOCLIENTIMPL_HPP
#define GAZEBOCLIENTIMPL_HPP

#include <string>
#include <memory>

namespace robotics {
namespace gazebotransport {
/// Action interface for implementing each Gazebo client action
class Action;

/// Client creates a TCP/IP client that connects to Server
class Client;

/// GazeboClient Interface for Gazebo actions
class GazeboClientImpl {
  public:
    /// constructor
    /**
    @param ipaddress       server ip address
    @param port            server port number
    @param timeout         time out for each Gazebo actions
    @exception             May throw if cannot connect to Server

    All actions in this class that interacts with remote Gazebo server
    will wait for the action to finish on server side, with maximum wait
    time specified by timeout
    */
    GazeboClientImpl(std::string const& ipaddress, uint16_t port, uint64_t timeout);

    /// act on the given Action
    /**
    @param actor           Action to take by the Gazebo client
    */
    void act(Action& actor);

    /// disconnect the client
    void shutdown();

  private:
    /// communication interface
    std::shared_ptr<robotics::gazebotransport::Client> m_client;

    /// action timeouts
    uint64_t m_timeout;
};
} // namespace gazebotransport
} // namespace robotics

#endif
