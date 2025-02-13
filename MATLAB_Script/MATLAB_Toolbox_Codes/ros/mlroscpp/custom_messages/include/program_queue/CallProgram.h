// Generated by gencpp from file program_queue/CallProgram.msg
// DO NOT EDIT!


#ifndef PROGRAM_QUEUE_MESSAGE_CALLPROGRAM_H
#define PROGRAM_QUEUE_MESSAGE_CALLPROGRAM_H

#include <ros/service_traits.h>


#include <program_queue/CallProgramRequest.h>
#include <program_queue/CallProgramResponse.h>


namespace program_queue
{

struct CallProgram
{

typedef CallProgramRequest Request;
typedef CallProgramResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct CallProgram
} // namespace program_queue


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::program_queue::CallProgram > {
  static const char* value()
  {
    return "5c879fd53e0d83f5c787f7fb59bfc760";
  }

  static const char* value(const ::program_queue::CallProgram&) { return value(); }
};

template<>
struct DataType< ::program_queue::CallProgram > {
  static const char* value()
  {
    return "program_queue/CallProgram";
  }

  static const char* value(const ::program_queue::CallProgram&) { return value(); }
};


// service_traits::MD5Sum< ::program_queue::CallProgramRequest> should match 
// service_traits::MD5Sum< ::program_queue::CallProgram > 
template<>
struct MD5Sum< ::program_queue::CallProgramRequest>
{
  static const char* value()
  {
    return MD5Sum< ::program_queue::CallProgram >::value();
  }
  static const char* value(const ::program_queue::CallProgramRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::program_queue::CallProgramRequest> should match 
// service_traits::DataType< ::program_queue::CallProgram > 
template<>
struct DataType< ::program_queue::CallProgramRequest>
{
  static const char* value()
  {
    return DataType< ::program_queue::CallProgram >::value();
  }
  static const char* value(const ::program_queue::CallProgramRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::program_queue::CallProgramResponse> should match 
// service_traits::MD5Sum< ::program_queue::CallProgram > 
template<>
struct MD5Sum< ::program_queue::CallProgramResponse>
{
  static const char* value()
  {
    return MD5Sum< ::program_queue::CallProgram >::value();
  }
  static const char* value(const ::program_queue::CallProgramResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::program_queue::CallProgramResponse> should match 
// service_traits::DataType< ::program_queue::CallProgram > 
template<>
struct DataType< ::program_queue::CallProgramResponse>
{
  static const char* value()
  {
    return DataType< ::program_queue::CallProgram >::value();
  }
  static const char* value(const ::program_queue::CallProgramResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // PROGRAM_QUEUE_MESSAGE_CALLPROGRAM_H
