// Generated by gencpp from file program_queue/ClearQueue.msg
// DO NOT EDIT!


#ifndef PROGRAM_QUEUE_MESSAGE_CLEARQUEUE_H
#define PROGRAM_QUEUE_MESSAGE_CLEARQUEUE_H

#include <ros/service_traits.h>


#include <program_queue/ClearQueueRequest.h>
#include <program_queue/ClearQueueResponse.h>


namespace program_queue
{

struct ClearQueue
{

typedef ClearQueueRequest Request;
typedef ClearQueueResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ClearQueue
} // namespace program_queue


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::program_queue::ClearQueue > {
  static const char* value()
  {
    return "14c7152ddd08a9946aaadd642a3c327d";
  }

  static const char* value(const ::program_queue::ClearQueue&) { return value(); }
};

template<>
struct DataType< ::program_queue::ClearQueue > {
  static const char* value()
  {
    return "program_queue/ClearQueue";
  }

  static const char* value(const ::program_queue::ClearQueue&) { return value(); }
};


// service_traits::MD5Sum< ::program_queue::ClearQueueRequest> should match 
// service_traits::MD5Sum< ::program_queue::ClearQueue > 
template<>
struct MD5Sum< ::program_queue::ClearQueueRequest>
{
  static const char* value()
  {
    return MD5Sum< ::program_queue::ClearQueue >::value();
  }
  static const char* value(const ::program_queue::ClearQueueRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::program_queue::ClearQueueRequest> should match 
// service_traits::DataType< ::program_queue::ClearQueue > 
template<>
struct DataType< ::program_queue::ClearQueueRequest>
{
  static const char* value()
  {
    return DataType< ::program_queue::ClearQueue >::value();
  }
  static const char* value(const ::program_queue::ClearQueueRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::program_queue::ClearQueueResponse> should match 
// service_traits::MD5Sum< ::program_queue::ClearQueue > 
template<>
struct MD5Sum< ::program_queue::ClearQueueResponse>
{
  static const char* value()
  {
    return MD5Sum< ::program_queue::ClearQueue >::value();
  }
  static const char* value(const ::program_queue::ClearQueueResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::program_queue::ClearQueueResponse> should match 
// service_traits::DataType< ::program_queue::ClearQueue > 
template<>
struct DataType< ::program_queue::ClearQueueResponse>
{
  static const char* value()
  {
    return DataType< ::program_queue::ClearQueue >::value();
  }
  static const char* value(const ::program_queue::ClearQueueResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // PROGRAM_QUEUE_MESSAGE_CLEARQUEUE_H