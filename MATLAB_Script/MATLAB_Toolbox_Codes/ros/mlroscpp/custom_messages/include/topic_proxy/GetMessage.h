// Generated by gencpp from file topic_proxy/GetMessage.msg
// DO NOT EDIT!


#ifndef TOPIC_PROXY_MESSAGE_GETMESSAGE_H
#define TOPIC_PROXY_MESSAGE_GETMESSAGE_H

#include <ros/service_traits.h>


#include <topic_proxy/GetMessageRequest.h>
#include <topic_proxy/GetMessageResponse.h>


namespace topic_proxy
{

struct GetMessage
{

typedef GetMessageRequest Request;
typedef GetMessageResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetMessage
} // namespace topic_proxy


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::topic_proxy::GetMessage > {
  static const char* value()
  {
    return "c0a1ee0be2bffb49a9ab9a06bfe444c3";
  }

  static const char* value(const ::topic_proxy::GetMessage&) { return value(); }
};

template<>
struct DataType< ::topic_proxy::GetMessage > {
  static const char* value()
  {
    return "topic_proxy/GetMessage";
  }

  static const char* value(const ::topic_proxy::GetMessage&) { return value(); }
};


// service_traits::MD5Sum< ::topic_proxy::GetMessageRequest> should match 
// service_traits::MD5Sum< ::topic_proxy::GetMessage > 
template<>
struct MD5Sum< ::topic_proxy::GetMessageRequest>
{
  static const char* value()
  {
    return MD5Sum< ::topic_proxy::GetMessage >::value();
  }
  static const char* value(const ::topic_proxy::GetMessageRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::topic_proxy::GetMessageRequest> should match 
// service_traits::DataType< ::topic_proxy::GetMessage > 
template<>
struct DataType< ::topic_proxy::GetMessageRequest>
{
  static const char* value()
  {
    return DataType< ::topic_proxy::GetMessage >::value();
  }
  static const char* value(const ::topic_proxy::GetMessageRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::topic_proxy::GetMessageResponse> should match 
// service_traits::MD5Sum< ::topic_proxy::GetMessage > 
template<>
struct MD5Sum< ::topic_proxy::GetMessageResponse>
{
  static const char* value()
  {
    return MD5Sum< ::topic_proxy::GetMessage >::value();
  }
  static const char* value(const ::topic_proxy::GetMessageResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::topic_proxy::GetMessageResponse> should match 
// service_traits::DataType< ::topic_proxy::GetMessage > 
template<>
struct DataType< ::topic_proxy::GetMessageResponse>
{
  static const char* value()
  {
    return DataType< ::topic_proxy::GetMessage >::value();
  }
  static const char* value(const ::topic_proxy::GetMessageResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // TOPIC_PROXY_MESSAGE_GETMESSAGE_H