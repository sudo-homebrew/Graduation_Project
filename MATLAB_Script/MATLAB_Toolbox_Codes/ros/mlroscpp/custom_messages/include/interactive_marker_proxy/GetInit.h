// Generated by gencpp from file interactive_marker_proxy/GetInit.msg
// DO NOT EDIT!


#ifndef INTERACTIVE_MARKER_PROXY_MESSAGE_GETINIT_H
#define INTERACTIVE_MARKER_PROXY_MESSAGE_GETINIT_H

#include <ros/service_traits.h>


#include <interactive_marker_proxy/GetInitRequest.h>
#include <interactive_marker_proxy/GetInitResponse.h>


namespace interactive_marker_proxy
{

struct GetInit
{

typedef GetInitRequest Request;
typedef GetInitResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetInit
} // namespace interactive_marker_proxy


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::interactive_marker_proxy::GetInit > {
  static const char* value()
  {
    return "4880f9fbacc796ee9d6c08994daf3e3c";
  }

  static const char* value(const ::interactive_marker_proxy::GetInit&) { return value(); }
};

template<>
struct DataType< ::interactive_marker_proxy::GetInit > {
  static const char* value()
  {
    return "interactive_marker_proxy/GetInit";
  }

  static const char* value(const ::interactive_marker_proxy::GetInit&) { return value(); }
};


// service_traits::MD5Sum< ::interactive_marker_proxy::GetInitRequest> should match 
// service_traits::MD5Sum< ::interactive_marker_proxy::GetInit > 
template<>
struct MD5Sum< ::interactive_marker_proxy::GetInitRequest>
{
  static const char* value()
  {
    return MD5Sum< ::interactive_marker_proxy::GetInit >::value();
  }
  static const char* value(const ::interactive_marker_proxy::GetInitRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::interactive_marker_proxy::GetInitRequest> should match 
// service_traits::DataType< ::interactive_marker_proxy::GetInit > 
template<>
struct DataType< ::interactive_marker_proxy::GetInitRequest>
{
  static const char* value()
  {
    return DataType< ::interactive_marker_proxy::GetInit >::value();
  }
  static const char* value(const ::interactive_marker_proxy::GetInitRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::interactive_marker_proxy::GetInitResponse> should match 
// service_traits::MD5Sum< ::interactive_marker_proxy::GetInit > 
template<>
struct MD5Sum< ::interactive_marker_proxy::GetInitResponse>
{
  static const char* value()
  {
    return MD5Sum< ::interactive_marker_proxy::GetInit >::value();
  }
  static const char* value(const ::interactive_marker_proxy::GetInitResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::interactive_marker_proxy::GetInitResponse> should match 
// service_traits::DataType< ::interactive_marker_proxy::GetInit > 
template<>
struct DataType< ::interactive_marker_proxy::GetInitResponse>
{
  static const char* value()
  {
    return DataType< ::interactive_marker_proxy::GetInit >::value();
  }
  static const char* value(const ::interactive_marker_proxy::GetInitResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // INTERACTIVE_MARKER_PROXY_MESSAGE_GETINIT_H