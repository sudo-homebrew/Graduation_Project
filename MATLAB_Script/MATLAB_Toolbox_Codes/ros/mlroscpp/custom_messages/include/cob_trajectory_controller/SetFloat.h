// Generated by gencpp from file cob_trajectory_controller/SetFloat.msg
// DO NOT EDIT!


#ifndef COB_TRAJECTORY_CONTROLLER_MESSAGE_SETFLOAT_H
#define COB_TRAJECTORY_CONTROLLER_MESSAGE_SETFLOAT_H

#include <ros/service_traits.h>


#include <cob_trajectory_controller/SetFloatRequest.h>
#include <cob_trajectory_controller/SetFloatResponse.h>


namespace cob_trajectory_controller
{

struct SetFloat
{

typedef SetFloatRequest Request;
typedef SetFloatResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetFloat
} // namespace cob_trajectory_controller


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::cob_trajectory_controller::SetFloat > {
  static const char* value()
  {
    return "c61d00936862523d63db9428e15b5b6e";
  }

  static const char* value(const ::cob_trajectory_controller::SetFloat&) { return value(); }
};

template<>
struct DataType< ::cob_trajectory_controller::SetFloat > {
  static const char* value()
  {
    return "cob_trajectory_controller/SetFloat";
  }

  static const char* value(const ::cob_trajectory_controller::SetFloat&) { return value(); }
};


// service_traits::MD5Sum< ::cob_trajectory_controller::SetFloatRequest> should match 
// service_traits::MD5Sum< ::cob_trajectory_controller::SetFloat > 
template<>
struct MD5Sum< ::cob_trajectory_controller::SetFloatRequest>
{
  static const char* value()
  {
    return MD5Sum< ::cob_trajectory_controller::SetFloat >::value();
  }
  static const char* value(const ::cob_trajectory_controller::SetFloatRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::cob_trajectory_controller::SetFloatRequest> should match 
// service_traits::DataType< ::cob_trajectory_controller::SetFloat > 
template<>
struct DataType< ::cob_trajectory_controller::SetFloatRequest>
{
  static const char* value()
  {
    return DataType< ::cob_trajectory_controller::SetFloat >::value();
  }
  static const char* value(const ::cob_trajectory_controller::SetFloatRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::cob_trajectory_controller::SetFloatResponse> should match 
// service_traits::MD5Sum< ::cob_trajectory_controller::SetFloat > 
template<>
struct MD5Sum< ::cob_trajectory_controller::SetFloatResponse>
{
  static const char* value()
  {
    return MD5Sum< ::cob_trajectory_controller::SetFloat >::value();
  }
  static const char* value(const ::cob_trajectory_controller::SetFloatResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::cob_trajectory_controller::SetFloatResponse> should match 
// service_traits::DataType< ::cob_trajectory_controller::SetFloat > 
template<>
struct DataType< ::cob_trajectory_controller::SetFloatResponse>
{
  static const char* value()
  {
    return DataType< ::cob_trajectory_controller::SetFloat >::value();
  }
  static const char* value(const ::cob_trajectory_controller::SetFloatResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // COB_TRAJECTORY_CONTROLLER_MESSAGE_SETFLOAT_H