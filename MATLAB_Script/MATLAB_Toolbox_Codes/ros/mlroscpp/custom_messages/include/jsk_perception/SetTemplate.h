// Generated by gencpp from file jsk_perception/SetTemplate.msg
// DO NOT EDIT!


#ifndef JSK_PERCEPTION_MESSAGE_SETTEMPLATE_H
#define JSK_PERCEPTION_MESSAGE_SETTEMPLATE_H

#include <ros/service_traits.h>


#include <jsk_perception/SetTemplateRequest.h>
#include <jsk_perception/SetTemplateResponse.h>


namespace jsk_perception
{

struct SetTemplate
{

typedef SetTemplateRequest Request;
typedef SetTemplateResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetTemplate
} // namespace jsk_perception


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::jsk_perception::SetTemplate > {
  static const char* value()
  {
    return "116fa80f27cbdfcd76d0b57a30ef79ec";
  }

  static const char* value(const ::jsk_perception::SetTemplate&) { return value(); }
};

template<>
struct DataType< ::jsk_perception::SetTemplate > {
  static const char* value()
  {
    return "jsk_perception/SetTemplate";
  }

  static const char* value(const ::jsk_perception::SetTemplate&) { return value(); }
};


// service_traits::MD5Sum< ::jsk_perception::SetTemplateRequest> should match 
// service_traits::MD5Sum< ::jsk_perception::SetTemplate > 
template<>
struct MD5Sum< ::jsk_perception::SetTemplateRequest>
{
  static const char* value()
  {
    return MD5Sum< ::jsk_perception::SetTemplate >::value();
  }
  static const char* value(const ::jsk_perception::SetTemplateRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::jsk_perception::SetTemplateRequest> should match 
// service_traits::DataType< ::jsk_perception::SetTemplate > 
template<>
struct DataType< ::jsk_perception::SetTemplateRequest>
{
  static const char* value()
  {
    return DataType< ::jsk_perception::SetTemplate >::value();
  }
  static const char* value(const ::jsk_perception::SetTemplateRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::jsk_perception::SetTemplateResponse> should match 
// service_traits::MD5Sum< ::jsk_perception::SetTemplate > 
template<>
struct MD5Sum< ::jsk_perception::SetTemplateResponse>
{
  static const char* value()
  {
    return MD5Sum< ::jsk_perception::SetTemplate >::value();
  }
  static const char* value(const ::jsk_perception::SetTemplateResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::jsk_perception::SetTemplateResponse> should match 
// service_traits::DataType< ::jsk_perception::SetTemplate > 
template<>
struct DataType< ::jsk_perception::SetTemplateResponse>
{
  static const char* value()
  {
    return DataType< ::jsk_perception::SetTemplate >::value();
  }
  static const char* value(const ::jsk_perception::SetTemplateResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // JSK_PERCEPTION_MESSAGE_SETTEMPLATE_H