// Generated by gencpp from file concert_msgs/ClientList.msg
// DO NOT EDIT!


#ifndef CONCERT_MSGS_MESSAGE_CLIENTLIST_H
#define CONCERT_MSGS_MESSAGE_CLIENTLIST_H

#include <ros/service_traits.h>


#include <concert_msgs/ClientListRequest.h>
#include <concert_msgs/ClientListResponse.h>


namespace concert_msgs
{

struct ClientList
{

typedef ClientListRequest Request;
typedef ClientListResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ClientList
} // namespace concert_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::concert_msgs::ClientList > {
  static const char* value()
  {
    return "9572afe4f30b1567d33f03cf448620a8";
  }

  static const char* value(const ::concert_msgs::ClientList&) { return value(); }
};

template<>
struct DataType< ::concert_msgs::ClientList > {
  static const char* value()
  {
    return "concert_msgs/ClientList";
  }

  static const char* value(const ::concert_msgs::ClientList&) { return value(); }
};


// service_traits::MD5Sum< ::concert_msgs::ClientListRequest> should match 
// service_traits::MD5Sum< ::concert_msgs::ClientList > 
template<>
struct MD5Sum< ::concert_msgs::ClientListRequest>
{
  static const char* value()
  {
    return MD5Sum< ::concert_msgs::ClientList >::value();
  }
  static const char* value(const ::concert_msgs::ClientListRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::concert_msgs::ClientListRequest> should match 
// service_traits::DataType< ::concert_msgs::ClientList > 
template<>
struct DataType< ::concert_msgs::ClientListRequest>
{
  static const char* value()
  {
    return DataType< ::concert_msgs::ClientList >::value();
  }
  static const char* value(const ::concert_msgs::ClientListRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::concert_msgs::ClientListResponse> should match 
// service_traits::MD5Sum< ::concert_msgs::ClientList > 
template<>
struct MD5Sum< ::concert_msgs::ClientListResponse>
{
  static const char* value()
  {
    return MD5Sum< ::concert_msgs::ClientList >::value();
  }
  static const char* value(const ::concert_msgs::ClientListResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::concert_msgs::ClientListResponse> should match 
// service_traits::DataType< ::concert_msgs::ClientList > 
template<>
struct DataType< ::concert_msgs::ClientListResponse>
{
  static const char* value()
  {
    return DataType< ::concert_msgs::ClientList >::value();
  }
  static const char* value(const ::concert_msgs::ClientListResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // CONCERT_MSGS_MESSAGE_CLIENTLIST_H