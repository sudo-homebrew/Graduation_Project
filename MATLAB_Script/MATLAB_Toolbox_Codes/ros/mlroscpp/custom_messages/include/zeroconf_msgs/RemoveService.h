// Generated by gencpp from file zeroconf_msgs/RemoveService.msg
// DO NOT EDIT!


#ifndef ZEROCONF_MSGS_MESSAGE_REMOVESERVICE_H
#define ZEROCONF_MSGS_MESSAGE_REMOVESERVICE_H

#include <ros/service_traits.h>


#include <zeroconf_msgs/RemoveServiceRequest.h>
#include <zeroconf_msgs/RemoveServiceResponse.h>


namespace zeroconf_msgs
{

struct RemoveService
{

typedef RemoveServiceRequest Request;
typedef RemoveServiceResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct RemoveService
} // namespace zeroconf_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::zeroconf_msgs::RemoveService > {
  static const char* value()
  {
    return "2ae0e420b2f58ab49b813cd81e42d4b3";
  }

  static const char* value(const ::zeroconf_msgs::RemoveService&) { return value(); }
};

template<>
struct DataType< ::zeroconf_msgs::RemoveService > {
  static const char* value()
  {
    return "zeroconf_msgs/RemoveService";
  }

  static const char* value(const ::zeroconf_msgs::RemoveService&) { return value(); }
};


// service_traits::MD5Sum< ::zeroconf_msgs::RemoveServiceRequest> should match 
// service_traits::MD5Sum< ::zeroconf_msgs::RemoveService > 
template<>
struct MD5Sum< ::zeroconf_msgs::RemoveServiceRequest>
{
  static const char* value()
  {
    return MD5Sum< ::zeroconf_msgs::RemoveService >::value();
  }
  static const char* value(const ::zeroconf_msgs::RemoveServiceRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::zeroconf_msgs::RemoveServiceRequest> should match 
// service_traits::DataType< ::zeroconf_msgs::RemoveService > 
template<>
struct DataType< ::zeroconf_msgs::RemoveServiceRequest>
{
  static const char* value()
  {
    return DataType< ::zeroconf_msgs::RemoveService >::value();
  }
  static const char* value(const ::zeroconf_msgs::RemoveServiceRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::zeroconf_msgs::RemoveServiceResponse> should match 
// service_traits::MD5Sum< ::zeroconf_msgs::RemoveService > 
template<>
struct MD5Sum< ::zeroconf_msgs::RemoveServiceResponse>
{
  static const char* value()
  {
    return MD5Sum< ::zeroconf_msgs::RemoveService >::value();
  }
  static const char* value(const ::zeroconf_msgs::RemoveServiceResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::zeroconf_msgs::RemoveServiceResponse> should match 
// service_traits::DataType< ::zeroconf_msgs::RemoveService > 
template<>
struct DataType< ::zeroconf_msgs::RemoveServiceResponse>
{
  static const char* value()
  {
    return DataType< ::zeroconf_msgs::RemoveService >::value();
  }
  static const char* value(const ::zeroconf_msgs::RemoveServiceResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ZEROCONF_MSGS_MESSAGE_REMOVESERVICE_H