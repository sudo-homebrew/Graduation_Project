// Generated by gencpp from file program_queue/Logout.msg
// DO NOT EDIT!


#ifndef PROGRAM_QUEUE_MESSAGE_LOGOUT_H
#define PROGRAM_QUEUE_MESSAGE_LOGOUT_H

#include <ros/service_traits.h>


#include <program_queue/LogoutRequest.h>
#include <program_queue/LogoutResponse.h>


namespace program_queue
{

struct Logout
{

typedef LogoutRequest Request;
typedef LogoutResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct Logout
} // namespace program_queue


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::program_queue::Logout > {
  static const char* value()
  {
    return "14c7152ddd08a9946aaadd642a3c327d";
  }

  static const char* value(const ::program_queue::Logout&) { return value(); }
};

template<>
struct DataType< ::program_queue::Logout > {
  static const char* value()
  {
    return "program_queue/Logout";
  }

  static const char* value(const ::program_queue::Logout&) { return value(); }
};


// service_traits::MD5Sum< ::program_queue::LogoutRequest> should match 
// service_traits::MD5Sum< ::program_queue::Logout > 
template<>
struct MD5Sum< ::program_queue::LogoutRequest>
{
  static const char* value()
  {
    return MD5Sum< ::program_queue::Logout >::value();
  }
  static const char* value(const ::program_queue::LogoutRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::program_queue::LogoutRequest> should match 
// service_traits::DataType< ::program_queue::Logout > 
template<>
struct DataType< ::program_queue::LogoutRequest>
{
  static const char* value()
  {
    return DataType< ::program_queue::Logout >::value();
  }
  static const char* value(const ::program_queue::LogoutRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::program_queue::LogoutResponse> should match 
// service_traits::MD5Sum< ::program_queue::Logout > 
template<>
struct MD5Sum< ::program_queue::LogoutResponse>
{
  static const char* value()
  {
    return MD5Sum< ::program_queue::Logout >::value();
  }
  static const char* value(const ::program_queue::LogoutResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::program_queue::LogoutResponse> should match 
// service_traits::DataType< ::program_queue::Logout > 
template<>
struct DataType< ::program_queue::LogoutResponse>
{
  static const char* value()
  {
    return DataType< ::program_queue::Logout >::value();
  }
  static const char* value(const ::program_queue::LogoutResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // PROGRAM_QUEUE_MESSAGE_LOGOUT_H
