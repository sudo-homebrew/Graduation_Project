// Generated by gencpp from file jsk_pcl_ros/SetPointCloud2.msg
// DO NOT EDIT!


#ifndef JSK_PCL_ROS_MESSAGE_SETPOINTCLOUD2_H
#define JSK_PCL_ROS_MESSAGE_SETPOINTCLOUD2_H

#include <ros/service_traits.h>


#include <jsk_pcl_ros/SetPointCloud2Request.h>
#include <jsk_pcl_ros/SetPointCloud2Response.h>


namespace jsk_pcl_ros
{

struct SetPointCloud2
{

typedef SetPointCloud2Request Request;
typedef SetPointCloud2Response Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetPointCloud2
} // namespace jsk_pcl_ros


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::jsk_pcl_ros::SetPointCloud2 > {
  static const char* value()
  {
    return "96cec5374164b3b3d1d7ef5d7628a7ed";
  }

  static const char* value(const ::jsk_pcl_ros::SetPointCloud2&) { return value(); }
};

template<>
struct DataType< ::jsk_pcl_ros::SetPointCloud2 > {
  static const char* value()
  {
    return "jsk_pcl_ros/SetPointCloud2";
  }

  static const char* value(const ::jsk_pcl_ros::SetPointCloud2&) { return value(); }
};


// service_traits::MD5Sum< ::jsk_pcl_ros::SetPointCloud2Request> should match 
// service_traits::MD5Sum< ::jsk_pcl_ros::SetPointCloud2 > 
template<>
struct MD5Sum< ::jsk_pcl_ros::SetPointCloud2Request>
{
  static const char* value()
  {
    return MD5Sum< ::jsk_pcl_ros::SetPointCloud2 >::value();
  }
  static const char* value(const ::jsk_pcl_ros::SetPointCloud2Request&)
  {
    return value();
  }
};

// service_traits::DataType< ::jsk_pcl_ros::SetPointCloud2Request> should match 
// service_traits::DataType< ::jsk_pcl_ros::SetPointCloud2 > 
template<>
struct DataType< ::jsk_pcl_ros::SetPointCloud2Request>
{
  static const char* value()
  {
    return DataType< ::jsk_pcl_ros::SetPointCloud2 >::value();
  }
  static const char* value(const ::jsk_pcl_ros::SetPointCloud2Request&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::jsk_pcl_ros::SetPointCloud2Response> should match 
// service_traits::MD5Sum< ::jsk_pcl_ros::SetPointCloud2 > 
template<>
struct MD5Sum< ::jsk_pcl_ros::SetPointCloud2Response>
{
  static const char* value()
  {
    return MD5Sum< ::jsk_pcl_ros::SetPointCloud2 >::value();
  }
  static const char* value(const ::jsk_pcl_ros::SetPointCloud2Response&)
  {
    return value();
  }
};

// service_traits::DataType< ::jsk_pcl_ros::SetPointCloud2Response> should match 
// service_traits::DataType< ::jsk_pcl_ros::SetPointCloud2 > 
template<>
struct DataType< ::jsk_pcl_ros::SetPointCloud2Response>
{
  static const char* value()
  {
    return DataType< ::jsk_pcl_ros::SetPointCloud2 >::value();
  }
  static const char* value(const ::jsk_pcl_ros::SetPointCloud2Response&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // JSK_PCL_ROS_MESSAGE_SETPOINTCLOUD2_H