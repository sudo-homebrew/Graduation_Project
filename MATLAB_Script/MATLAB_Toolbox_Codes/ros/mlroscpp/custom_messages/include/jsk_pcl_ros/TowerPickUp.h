// Generated by gencpp from file jsk_pcl_ros/TowerPickUp.msg
// DO NOT EDIT!


#ifndef JSK_PCL_ROS_MESSAGE_TOWERPICKUP_H
#define JSK_PCL_ROS_MESSAGE_TOWERPICKUP_H

#include <ros/service_traits.h>


#include <jsk_pcl_ros/TowerPickUpRequest.h>
#include <jsk_pcl_ros/TowerPickUpResponse.h>


namespace jsk_pcl_ros
{

struct TowerPickUp
{

typedef TowerPickUpRequest Request;
typedef TowerPickUpResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct TowerPickUp
} // namespace jsk_pcl_ros


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::jsk_pcl_ros::TowerPickUp > {
  static const char* value()
  {
    return "e8bd24109f26b6d833bc4570d67d71c9";
  }

  static const char* value(const ::jsk_pcl_ros::TowerPickUp&) { return value(); }
};

template<>
struct DataType< ::jsk_pcl_ros::TowerPickUp > {
  static const char* value()
  {
    return "jsk_pcl_ros/TowerPickUp";
  }

  static const char* value(const ::jsk_pcl_ros::TowerPickUp&) { return value(); }
};


// service_traits::MD5Sum< ::jsk_pcl_ros::TowerPickUpRequest> should match 
// service_traits::MD5Sum< ::jsk_pcl_ros::TowerPickUp > 
template<>
struct MD5Sum< ::jsk_pcl_ros::TowerPickUpRequest>
{
  static const char* value()
  {
    return MD5Sum< ::jsk_pcl_ros::TowerPickUp >::value();
  }
  static const char* value(const ::jsk_pcl_ros::TowerPickUpRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::jsk_pcl_ros::TowerPickUpRequest> should match 
// service_traits::DataType< ::jsk_pcl_ros::TowerPickUp > 
template<>
struct DataType< ::jsk_pcl_ros::TowerPickUpRequest>
{
  static const char* value()
  {
    return DataType< ::jsk_pcl_ros::TowerPickUp >::value();
  }
  static const char* value(const ::jsk_pcl_ros::TowerPickUpRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::jsk_pcl_ros::TowerPickUpResponse> should match 
// service_traits::MD5Sum< ::jsk_pcl_ros::TowerPickUp > 
template<>
struct MD5Sum< ::jsk_pcl_ros::TowerPickUpResponse>
{
  static const char* value()
  {
    return MD5Sum< ::jsk_pcl_ros::TowerPickUp >::value();
  }
  static const char* value(const ::jsk_pcl_ros::TowerPickUpResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::jsk_pcl_ros::TowerPickUpResponse> should match 
// service_traits::DataType< ::jsk_pcl_ros::TowerPickUp > 
template<>
struct DataType< ::jsk_pcl_ros::TowerPickUpResponse>
{
  static const char* value()
  {
    return DataType< ::jsk_pcl_ros::TowerPickUp >::value();
  }
  static const char* value(const ::jsk_pcl_ros::TowerPickUpResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // JSK_PCL_ROS_MESSAGE_TOWERPICKUP_H