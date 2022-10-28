/* Copyright 2019 The MathWorks, Inc. */
#include "gazebotransport/gazeboserver/gazebosensors/GazeboSensorSubscriber.hpp"

#include "gazebo/sensors/SensorsIface.hh"
#include "gazebo/physics/World.hh"
#include "boost/algorithm/string.hpp"

#include <limits>
namespace robotics {
namespace gazebotransport {
GazeboSensorSubscriber::GazeboSensorSubscriber(gazebo::physics::WorldPtr world,
                                               std::string const& topic)
    : m_world(world)
    , m_topicName(topic)
    , m_sensorName()
    , m_updateInterval(std::numeric_limits<double>::infinity())
    , m_mutex()
    , m_lastUpdateTime(-1)
    , m_lastMessage() {
    // For Gazebo Camera, IMU and Laser sensors
    // topic is in format /gazebo/$scopedSensorName/$topicName
    // we want to remove the leading /gazebo/
    // and we want to remove the ending /$topicName
    auto start = topic.find_first_of("/");
    auto end = topic.find_last_of("/");

    m_sensorName = m_topicName.substr(start + 1, end - start - 1);
    start = m_sensorName.find_first_of("/");
    m_sensorName = m_sensorName.substr(start + 1);

    boost::replace_all(m_sensorName, "/", "::");

    // fill the sensor update interval
    auto sensor = gazebo::sensors::get_sensor(m_sensorName);
    if (sensor && sensor->UpdateRate() != 0) {
        m_updateInterval = 1 / sensor->UpdateRate();
        m_lastUpdateTime = -m_updateInterval;
    }

    // initialize the message buffer
    resetMessageBuffer();
}

GazeboSensorSubscriber::~GazeboSensorSubscriber() {
}

std::string GazeboSensorSubscriber::getTopicName() const {
    return m_topicName;
}

std::string GazeboSensorSubscriber::getSensorName() const {
    return m_sensorName;
}

mw::internal::robotics::gazebotransport::Packet GazeboSensorSubscriber::getLatestMessage() {
    // collect the time difference between current simulation and last sensor update
    // if the difference is greater than the update period, wait for sensor update to
    // refresh itself
    double diffTime;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        diffTime = (m_world->SimTime() - m_lastUpdateTime).Double();
    }

    // wait for at most 500 ms seconds, divided into 50 queries
    size_t idx = 0;
    while (diffTime >= m_updateInterval && idx < 50) {
        idx++;
        // during this 5 ms second sleep, the updateMessage could proceed to
        // update the camera image
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        std::lock_guard<std::mutex> lock(m_mutex);
        diffTime = (m_world->SimTime() - m_lastUpdateTime).Double();
    }



    if (diffTime < 0) {
        // reset message buffer if it is coming from future
        // which would be caused by user starting Gazebo co-simulation
        // from a future simulation time point
        resetMessageBuffer();
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    return m_lastMessage;
}

void GazeboSensorSubscriber::init(gazebo::transport::NodePtr node) {
    // initialize the message buffer
    resetMessageBuffer();

    // call initImpl from actual sensor implementations
    initImpl(node);
}

void GazeboSensorSubscriber::resetMessageBuffer() {
    std::lock_guard<std::mutex> lock(m_mutex);
    // initialize the message buffer
    m_lastMessage.mutable_header()->set_id(
        mw::internal::robotics::gazebotransport::PacketHeader_MsgID_STATUS);
    m_lastMessage.mutable_header()->mutable_time_stamp()->set_seconds(0);
    m_lastMessage.mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
    m_lastMessage.set_status(
        mw::internal::robotics::gazebotransport::Packet_CoSimError_TOPIC_NAME_INVALID);
}

} // namespace gazebotransport
} // namespace robotics
