// Generated by gencpp from file nao_msgs/SetArmsEnabled.msg
// DO NOT EDIT!


#ifndef NAO_MSGS_MESSAGE_SETARMSENABLED_H
#define NAO_MSGS_MESSAGE_SETARMSENABLED_H

#include <ros/service_traits.h>


#include <nao_msgs/SetArmsEnabledRequest.h>
#include <nao_msgs/SetArmsEnabledResponse.h>


namespace nao_msgs
{

struct SetArmsEnabled
{

typedef SetArmsEnabledRequest Request;
typedef SetArmsEnabledResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetArmsEnabled
} // namespace nao_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::nao_msgs::SetArmsEnabled > {
  static const char* value()
  {
    return "4da9069facca935244c3405e288ba555";
  }

  static const char* value(const ::nao_msgs::SetArmsEnabled&) { return value(); }
};

template<>
struct DataType< ::nao_msgs::SetArmsEnabled > {
  static const char* value()
  {
    return "nao_msgs/SetArmsEnabled";
  }

  static const char* value(const ::nao_msgs::SetArmsEnabled&) { return value(); }
};


// service_traits::MD5Sum< ::nao_msgs::SetArmsEnabledRequest> should match 
// service_traits::MD5Sum< ::nao_msgs::SetArmsEnabled > 
template<>
struct MD5Sum< ::nao_msgs::SetArmsEnabledRequest>
{
  static const char* value()
  {
    return MD5Sum< ::nao_msgs::SetArmsEnabled >::value();
  }
  static const char* value(const ::nao_msgs::SetArmsEnabledRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::nao_msgs::SetArmsEnabledRequest> should match 
// service_traits::DataType< ::nao_msgs::SetArmsEnabled > 
template<>
struct DataType< ::nao_msgs::SetArmsEnabledRequest>
{
  static const char* value()
  {
    return DataType< ::nao_msgs::SetArmsEnabled >::value();
  }
  static const char* value(const ::nao_msgs::SetArmsEnabledRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::nao_msgs::SetArmsEnabledResponse> should match 
// service_traits::MD5Sum< ::nao_msgs::SetArmsEnabled > 
template<>
struct MD5Sum< ::nao_msgs::SetArmsEnabledResponse>
{
  static const char* value()
  {
    return MD5Sum< ::nao_msgs::SetArmsEnabled >::value();
  }
  static const char* value(const ::nao_msgs::SetArmsEnabledResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::nao_msgs::SetArmsEnabledResponse> should match 
// service_traits::DataType< ::nao_msgs::SetArmsEnabled > 
template<>
struct DataType< ::nao_msgs::SetArmsEnabledResponse>
{
  static const char* value()
  {
    return DataType< ::nao_msgs::SetArmsEnabled >::value();
  }
  static const char* value(const ::nao_msgs::SetArmsEnabledResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // NAO_MSGS_MESSAGE_SETARMSENABLED_H