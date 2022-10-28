/* Copyright 2019 The MathWorks, Inc. */
#ifndef GENERIC_MESSAGE_TYPE_HPP
#define GENERIC_MESSAGE_TYPE_HPP

#include <vector>
#include <string>

namespace robotics {
namespace gazebotransport {
enum GenericMessageType { IMAGE = 0, FUSED_IMU, LIDAR_SCAN };

static const std::vector<std::string> GenericMessageTypeStr = {
    "gazebo_msgs/Image", "gazebo_msgs/IMU", "gazebo_msgs/LaserScan"};
} // namespace gazebotransport
} // namespace robotics

#endif
