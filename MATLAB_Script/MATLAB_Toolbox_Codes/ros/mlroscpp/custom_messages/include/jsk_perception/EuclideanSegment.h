// Generated by gencpp from file jsk_perception/EuclideanSegment.msg
// DO NOT EDIT!


#ifndef JSK_PERCEPTION_MESSAGE_EUCLIDEANSEGMENT_H
#define JSK_PERCEPTION_MESSAGE_EUCLIDEANSEGMENT_H

#include <ros/service_traits.h>


#include <jsk_perception/EuclideanSegmentRequest.h>
#include <jsk_perception/EuclideanSegmentResponse.h>


namespace jsk_perception
{

struct EuclideanSegment
{

typedef EuclideanSegmentRequest Request;
typedef EuclideanSegmentResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct EuclideanSegment
} // namespace jsk_perception


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::jsk_perception::EuclideanSegment > {
  static const char* value()
  {
    return "7210bbdf9078b61fce51942a9f673096";
  }

  static const char* value(const ::jsk_perception::EuclideanSegment&) { return value(); }
};

template<>
struct DataType< ::jsk_perception::EuclideanSegment > {
  static const char* value()
  {
    return "jsk_perception/EuclideanSegment";
  }

  static const char* value(const ::jsk_perception::EuclideanSegment&) { return value(); }
};


// service_traits::MD5Sum< ::jsk_perception::EuclideanSegmentRequest> should match 
// service_traits::MD5Sum< ::jsk_perception::EuclideanSegment > 
template<>
struct MD5Sum< ::jsk_perception::EuclideanSegmentRequest>
{
  static const char* value()
  {
    return MD5Sum< ::jsk_perception::EuclideanSegment >::value();
  }
  static const char* value(const ::jsk_perception::EuclideanSegmentRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::jsk_perception::EuclideanSegmentRequest> should match 
// service_traits::DataType< ::jsk_perception::EuclideanSegment > 
template<>
struct DataType< ::jsk_perception::EuclideanSegmentRequest>
{
  static const char* value()
  {
    return DataType< ::jsk_perception::EuclideanSegment >::value();
  }
  static const char* value(const ::jsk_perception::EuclideanSegmentRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::jsk_perception::EuclideanSegmentResponse> should match 
// service_traits::MD5Sum< ::jsk_perception::EuclideanSegment > 
template<>
struct MD5Sum< ::jsk_perception::EuclideanSegmentResponse>
{
  static const char* value()
  {
    return MD5Sum< ::jsk_perception::EuclideanSegment >::value();
  }
  static const char* value(const ::jsk_perception::EuclideanSegmentResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::jsk_perception::EuclideanSegmentResponse> should match 
// service_traits::DataType< ::jsk_perception::EuclideanSegment > 
template<>
struct DataType< ::jsk_perception::EuclideanSegmentResponse>
{
  static const char* value()
  {
    return DataType< ::jsk_perception::EuclideanSegment >::value();
  }
  static const char* value(const ::jsk_perception::EuclideanSegmentResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // JSK_PERCEPTION_MESSAGE_EUCLIDEANSEGMENT_H