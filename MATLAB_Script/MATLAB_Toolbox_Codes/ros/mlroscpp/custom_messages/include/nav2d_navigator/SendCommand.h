// Generated by gencpp from file nav2d_navigator/SendCommand.msg
// DO NOT EDIT!


#ifndef NAV2D_NAVIGATOR_MESSAGE_SENDCOMMAND_H
#define NAV2D_NAVIGATOR_MESSAGE_SENDCOMMAND_H

#include <ros/service_traits.h>


#include <nav2d_navigator/SendCommandRequest.h>
#include <nav2d_navigator/SendCommandResponse.h>


namespace nav2d_navigator
{

struct SendCommand
{

typedef SendCommandRequest Request;
typedef SendCommandResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SendCommand
} // namespace nav2d_navigator


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::nav2d_navigator::SendCommand > {
  static const char* value()
  {
    return "ceebad186efa4eee5e8f54bd3b883361";
  }

  static const char* value(const ::nav2d_navigator::SendCommand&) { return value(); }
};

template<>
struct DataType< ::nav2d_navigator::SendCommand > {
  static const char* value()
  {
    return "nav2d_navigator/SendCommand";
  }

  static const char* value(const ::nav2d_navigator::SendCommand&) { return value(); }
};


// service_traits::MD5Sum< ::nav2d_navigator::SendCommandRequest> should match 
// service_traits::MD5Sum< ::nav2d_navigator::SendCommand > 
template<>
struct MD5Sum< ::nav2d_navigator::SendCommandRequest>
{
  static const char* value()
  {
    return MD5Sum< ::nav2d_navigator::SendCommand >::value();
  }
  static const char* value(const ::nav2d_navigator::SendCommandRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::nav2d_navigator::SendCommandRequest> should match 
// service_traits::DataType< ::nav2d_navigator::SendCommand > 
template<>
struct DataType< ::nav2d_navigator::SendCommandRequest>
{
  static const char* value()
  {
    return DataType< ::nav2d_navigator::SendCommand >::value();
  }
  static const char* value(const ::nav2d_navigator::SendCommandRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::nav2d_navigator::SendCommandResponse> should match 
// service_traits::MD5Sum< ::nav2d_navigator::SendCommand > 
template<>
struct MD5Sum< ::nav2d_navigator::SendCommandResponse>
{
  static const char* value()
  {
    return MD5Sum< ::nav2d_navigator::SendCommand >::value();
  }
  static const char* value(const ::nav2d_navigator::SendCommandResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::nav2d_navigator::SendCommandResponse> should match 
// service_traits::DataType< ::nav2d_navigator::SendCommand > 
template<>
struct DataType< ::nav2d_navigator::SendCommandResponse>
{
  static const char* value()
  {
    return DataType< ::nav2d_navigator::SendCommand >::value();
  }
  static const char* value(const ::nav2d_navigator::SendCommandResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // NAV2D_NAVIGATOR_MESSAGE_SENDCOMMAND_H