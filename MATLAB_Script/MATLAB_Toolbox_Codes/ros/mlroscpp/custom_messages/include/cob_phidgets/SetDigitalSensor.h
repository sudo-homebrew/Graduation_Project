// Generated by gencpp from file cob_phidgets/SetDigitalSensor.msg
// DO NOT EDIT!


#ifndef COB_PHIDGETS_MESSAGE_SETDIGITALSENSOR_H
#define COB_PHIDGETS_MESSAGE_SETDIGITALSENSOR_H

#include <ros/service_traits.h>


#include <cob_phidgets/SetDigitalSensorRequest.h>
#include <cob_phidgets/SetDigitalSensorResponse.h>


namespace cob_phidgets
{

struct SetDigitalSensor
{

typedef SetDigitalSensorRequest Request;
typedef SetDigitalSensorResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetDigitalSensor
} // namespace cob_phidgets


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::cob_phidgets::SetDigitalSensor > {
  static const char* value()
  {
    return "1ddd624f449abe5e039f3478b75cb4d7";
  }

  static const char* value(const ::cob_phidgets::SetDigitalSensor&) { return value(); }
};

template<>
struct DataType< ::cob_phidgets::SetDigitalSensor > {
  static const char* value()
  {
    return "cob_phidgets/SetDigitalSensor";
  }

  static const char* value(const ::cob_phidgets::SetDigitalSensor&) { return value(); }
};


// service_traits::MD5Sum< ::cob_phidgets::SetDigitalSensorRequest> should match
// service_traits::MD5Sum< ::cob_phidgets::SetDigitalSensor >
template<>
struct MD5Sum< ::cob_phidgets::SetDigitalSensorRequest>
{
  static const char* value()
  {
    return MD5Sum< ::cob_phidgets::SetDigitalSensor >::value();
  }
  static const char* value(const ::cob_phidgets::SetDigitalSensorRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::cob_phidgets::SetDigitalSensorRequest> should match
// service_traits::DataType< ::cob_phidgets::SetDigitalSensor >
template<>
struct DataType< ::cob_phidgets::SetDigitalSensorRequest>
{
  static const char* value()
  {
    return DataType< ::cob_phidgets::SetDigitalSensor >::value();
  }
  static const char* value(const ::cob_phidgets::SetDigitalSensorRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::cob_phidgets::SetDigitalSensorResponse> should match
// service_traits::MD5Sum< ::cob_phidgets::SetDigitalSensor >
template<>
struct MD5Sum< ::cob_phidgets::SetDigitalSensorResponse>
{
  static const char* value()
  {
    return MD5Sum< ::cob_phidgets::SetDigitalSensor >::value();
  }
  static const char* value(const ::cob_phidgets::SetDigitalSensorResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::cob_phidgets::SetDigitalSensorResponse> should match
// service_traits::DataType< ::cob_phidgets::SetDigitalSensor >
template<>
struct DataType< ::cob_phidgets::SetDigitalSensorResponse>
{
  static const char* value()
  {
    return DataType< ::cob_phidgets::SetDigitalSensor >::value();
  }
  static const char* value(const ::cob_phidgets::SetDigitalSensorResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // COB_PHIDGETS_MESSAGE_SETDIGITALSENSOR_H