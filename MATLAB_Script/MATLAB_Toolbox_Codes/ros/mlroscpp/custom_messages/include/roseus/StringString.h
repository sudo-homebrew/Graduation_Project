// Generated by gencpp from file roseus/StringString.msg
// DO NOT EDIT!


#ifndef ROSEUS_MESSAGE_STRINGSTRING_H
#define ROSEUS_MESSAGE_STRINGSTRING_H

#include <ros/service_traits.h>


#include <roseus/StringStringRequest.h>
#include <roseus/StringStringResponse.h>


namespace roseus
{

struct StringString
{

typedef StringStringRequest Request;
typedef StringStringResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct StringString
} // namespace roseus


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::roseus::StringString > {
  static const char* value()
  {
    return "671f8e4998eaec79f1c47e339dfd527b";
  }

  static const char* value(const ::roseus::StringString&) { return value(); }
};

template<>
struct DataType< ::roseus::StringString > {
  static const char* value()
  {
    return "roseus/StringString";
  }

  static const char* value(const ::roseus::StringString&) { return value(); }
};


// service_traits::MD5Sum< ::roseus::StringStringRequest> should match
// service_traits::MD5Sum< ::roseus::StringString >
template<>
struct MD5Sum< ::roseus::StringStringRequest>
{
  static const char* value()
  {
    return MD5Sum< ::roseus::StringString >::value();
  }
  static const char* value(const ::roseus::StringStringRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::roseus::StringStringRequest> should match
// service_traits::DataType< ::roseus::StringString >
template<>
struct DataType< ::roseus::StringStringRequest>
{
  static const char* value()
  {
    return DataType< ::roseus::StringString >::value();
  }
  static const char* value(const ::roseus::StringStringRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::roseus::StringStringResponse> should match
// service_traits::MD5Sum< ::roseus::StringString >
template<>
struct MD5Sum< ::roseus::StringStringResponse>
{
  static const char* value()
  {
    return MD5Sum< ::roseus::StringString >::value();
  }
  static const char* value(const ::roseus::StringStringResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::roseus::StringStringResponse> should match
// service_traits::DataType< ::roseus::StringString >
template<>
struct DataType< ::roseus::StringStringResponse>
{
  static const char* value()
  {
    return DataType< ::roseus::StringString >::value();
  }
  static const char* value(const ::roseus::StringStringResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ROSEUS_MESSAGE_STRINGSTRING_H