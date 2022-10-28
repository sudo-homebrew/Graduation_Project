/* Copyright 2019 The MathWorks, Inc. */
#ifndef GENERIC_MESSAGE_HANDLER_HPP
#define GENERIC_MESSAGE_HANDLER_HPP

#include "gazebotransport/gazeboclient/GenericMessageType.hpp"

#include "gazebotransport/mw.internal.robotics.gazebotransport.CoSimMsgs.pb.h"

#include <string>

namespace robotics {
namespace gazebotransport {
/// interface class for generating subscribe and requesting messages for generic Gazebo messages
class GenericMessageHandler {
  public:
    /// virtual destructor
    virtual ~GenericMessageHandler() = default;

    /// setup subscribe packet
    /**
    @param[in] topicName               Topic name to fill into the subscribe message
    @param[out] packet                 Packet to fill in the subscribe information
    */
    virtual void setupSubscribe(std::string const& topicName,
                                mw::internal::robotics::gazebotransport::Packet& packet) = 0;

    /// setup request packet
    /**
    @param[in] topicName               Topic name to fill into the request
    @param[out] packet                 Packet to fill in the request information
    */
    virtual void setupRequest(std::string const& topicName,
                              mw::internal::robotics::gazebotransport::Packet& packet) = 0;

    /// check whether the reply contains message that matches the one requested
    /**
    @param[in] reply                   Message that replied from Gazebo server
    @return                            Whether the reply match the requested message
    */
    virtual bool matchRequest(mw::internal::robotics::gazebotransport::Packet const& reply) = 0;

    /// fill the input packet with zeros or empty strings for its data field
    /**
    @param[in] input                   Pointer to message container to be initialized
    @return                            Pointer to message container initialized with zero values
    */
    virtual std::shared_ptr<mw::internal::robotics::gazebotransport::Packet> initalizeMessage(
        std::shared_ptr<mw::internal::robotics::gazebotransport::Packet>&& input) = 0;
};

/// Concrete requester for image message
class ImageMessageHandler : public GenericMessageHandler {
  public:
    virtual void setupSubscribe(std::string const& topicName,
                                mw::internal::robotics::gazebotransport::Packet& packet) override {
        // fill the message id
        packet.mutable_header()->set_id(mw::internal::robotics::gazebotransport::
                                            PacketHeader_MsgID::PacketHeader_MsgID_SUBSCRIBE_IMAGE);
        // fill the topic name
        packet.mutable_subscribe_image()->set_topic_name(topicName);
    }

    void setupRequest(std::string const& topicName,
                      mw::internal::robotics::gazebotransport::Packet& packet) override {
        // fill the message id
        packet.mutable_header()->set_id(mw::internal::robotics::gazebotransport::
                                            PacketHeader_MsgID::PacketHeader_MsgID_REQUEST_IMAGE);
        // fill the topic name
        packet.mutable_request_image()->set_topic_name(topicName);
    }

    bool matchRequest(mw::internal::robotics::gazebotransport::Packet const& reply) override {
        return reply.has_image();
    }

    std::shared_ptr<mw::internal::robotics::gazebotransport::Packet> initalizeMessage(
        std::shared_ptr<mw::internal::robotics::gazebotransport::Packet>&& input) override {
        input->mutable_header()->set_id(
            mw::internal::robotics::gazebotransport::PacketHeader_MsgID::PacketHeader_MsgID_IMAGE);
        input->mutable_header()->mutable_time_stamp()->set_seconds(0);
        input->mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        input->mutable_image()->set_width(0);
        input->mutable_image()->set_height(0);
        input->mutable_image()->set_data("");
        input->mutable_image()->set_data_type("");
        return std::move(input);
    }
};

/// Concrete requester for Fused IMU message
class FusedIMUMessageHandler : public GenericMessageHandler {
  public:
    virtual void setupSubscribe(std::string const& topicName,
                                mw::internal::robotics::gazebotransport::Packet& packet) override {
        // fill the message id
        packet.mutable_header()->set_id(mw::internal::robotics::gazebotransport::
                                            PacketHeader_MsgID::PacketHeader_MsgID_SUBSCRIBE_IMU);
        // fill the topic name
        packet.mutable_subscribe_imu()->set_topic_name(topicName);
    }

    void setupRequest(std::string const& topicName,
                      mw::internal::robotics::gazebotransport::Packet& packet) override {
        // fill the message id
        packet.mutable_header()->set_id(mw::internal::robotics::gazebotransport::
                                            PacketHeader_MsgID::PacketHeader_MsgID_REQUEST_IMU);
        // fill the topic name
        packet.mutable_request_imu()->set_topic_name(topicName);
    }

    bool matchRequest(mw::internal::robotics::gazebotransport::Packet const& reply) override {
        return reply.has_imu_data();
    }

    std::shared_ptr<mw::internal::robotics::gazebotransport::Packet> initalizeMessage(
        std::shared_ptr<mw::internal::robotics::gazebotransport::Packet>&& input) override {
        input->mutable_header()->set_id(mw::internal::robotics::gazebotransport::
                                            PacketHeader_MsgID::PacketHeader_MsgID_IMU_DATA);
        input->mutable_header()->mutable_time_stamp()->set_seconds(0);
        input->mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        input->mutable_imu_data()->mutable_linear_acceleration()->set_x(0);
        input->mutable_imu_data()->mutable_linear_acceleration()->set_y(0);
        input->mutable_imu_data()->mutable_linear_acceleration()->set_z(0);
        input->mutable_imu_data()->mutable_angular_velocity()->set_x(0);
        input->mutable_imu_data()->mutable_angular_velocity()->set_y(0);
        input->mutable_imu_data()->mutable_angular_velocity()->set_z(0);
        input->mutable_imu_data()->mutable_orientation()->set_x(0);
        input->mutable_imu_data()->mutable_orientation()->set_y(0);
        input->mutable_imu_data()->mutable_orientation()->set_z(0);
        input->mutable_imu_data()->mutable_orientation()->set_w(0);
        return std::move(input);
    }
};

/// Concrete requester for Lidar scan message
class LidarScanMessageHandler : public GenericMessageHandler {
  public:
    virtual void setupSubscribe(std::string const& topicName,
                                mw::internal::robotics::gazebotransport::Packet& packet) override {
        // fill the message id
        packet.mutable_header()->set_id(mw::internal::robotics::gazebotransport::
                                            PacketHeader_MsgID::PacketHeader_MsgID_SUBSCRIBE_LASER);
        // fill the topic name
        packet.mutable_subscribe_laser()->set_topic_name(topicName);
    }

    void setupRequest(std::string const& topicName,
                      mw::internal::robotics::gazebotransport::Packet& packet) override {
        // fill the message id
        packet.mutable_header()->set_id(mw::internal::robotics::gazebotransport::
                                            PacketHeader_MsgID::PacketHeader_MsgID_REQUEST_LASER);
        // fill the topic name
        packet.mutable_request_laser()->set_topic_name(topicName);
    }

    bool matchRequest(mw::internal::robotics::gazebotransport::Packet const& reply) override {
        return reply.has_laser_data();
    }

    std::shared_ptr<mw::internal::robotics::gazebotransport::Packet> initalizeMessage(
        std::shared_ptr<mw::internal::robotics::gazebotransport::Packet>&& input) override {
        input->mutable_header()->set_id(mw::internal::robotics::gazebotransport::
                                            PacketHeader_MsgID::PacketHeader_MsgID_LASER_DATA);
        input->mutable_header()->mutable_time_stamp()->set_seconds(0);
        input->mutable_header()->mutable_time_stamp()->set_nano_seconds(0);
        input->mutable_laser_data()->set_angle_min(0);
        input->mutable_laser_data()->set_angle_max(0);
        input->mutable_laser_data()->set_angle_step(0);
        input->mutable_laser_data()->set_vertical_angle_min(0);
        input->mutable_laser_data()->set_vertical_angle_max(0);
        input->mutable_laser_data()->set_vertical_angle_step(0);
        input->mutable_laser_data()->set_range_min(0);
        input->mutable_laser_data()->set_range_max(0);
        input->mutable_laser_data()->set_count(0);
        input->mutable_laser_data()->add_range(0);
        input->mutable_laser_data()->add_intensities(0);
        return std::move(input);
    }
};

} // namespace gazebotransport
} // namespace robotics

#endif
