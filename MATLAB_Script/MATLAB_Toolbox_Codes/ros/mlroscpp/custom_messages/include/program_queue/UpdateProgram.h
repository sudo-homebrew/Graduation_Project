// Generated by gencpp from file program_queue/UpdateProgram.msg
// DO NOT EDIT!


#ifndef PROGRAM_QUEUE_MESSAGE_UPDATEPROGRAM_H
#define PROGRAM_QUEUE_MESSAGE_UPDATEPROGRAM_H

#include <ros/service_traits.h>


#include <program_queue/UpdateProgramRequest.h>
#include <program_queue/UpdateProgramResponse.h>


namespace program_queue
{

struct UpdateProgram
{

typedef UpdateProgramRequest Request;
typedef UpdateProgramResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct UpdateProgram
} // namespace program_queue


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::program_queue::UpdateProgram > {
  static const char* value()
  {
    return "95a7f594407f333113b2a1fe09071e8c";
  }

  static const char* value(const ::program_queue::UpdateProgram&) { return value(); }
};

template<>
struct DataType< ::program_queue::UpdateProgram > {
  static const char* value()
  {
    return "program_queue/UpdateProgram";
  }

  static const char* value(const ::program_queue::UpdateProgram&) { return value(); }
};


// service_traits::MD5Sum< ::program_queue::UpdateProgramRequest> should match 
// service_traits::MD5Sum< ::program_queue::UpdateProgram > 
template<>
struct MD5Sum< ::program_queue::UpdateProgramRequest>
{
  static const char* value()
  {
    return MD5Sum< ::program_queue::UpdateProgram >::value();
  }
  static const char* value(const ::program_queue::UpdateProgramRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::program_queue::UpdateProgramRequest> should match 
// service_traits::DataType< ::program_queue::UpdateProgram > 
template<>
struct DataType< ::program_queue::UpdateProgramRequest>
{
  static const char* value()
  {
    return DataType< ::program_queue::UpdateProgram >::value();
  }
  static const char* value(const ::program_queue::UpdateProgramRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::program_queue::UpdateProgramResponse> should match 
// service_traits::MD5Sum< ::program_queue::UpdateProgram > 
template<>
struct MD5Sum< ::program_queue::UpdateProgramResponse>
{
  static const char* value()
  {
    return MD5Sum< ::program_queue::UpdateProgram >::value();
  }
  static const char* value(const ::program_queue::UpdateProgramResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::program_queue::UpdateProgramResponse> should match 
// service_traits::DataType< ::program_queue::UpdateProgram > 
template<>
struct DataType< ::program_queue::UpdateProgramResponse>
{
  static const char* value()
  {
    return DataType< ::program_queue::UpdateProgram >::value();
  }
  static const char* value(const ::program_queue::UpdateProgramResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // PROGRAM_QUEUE_MESSAGE_UPDATEPROGRAM_H