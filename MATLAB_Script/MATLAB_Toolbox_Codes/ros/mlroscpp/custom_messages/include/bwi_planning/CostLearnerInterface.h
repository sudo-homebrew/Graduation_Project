// Generated by gencpp from file bwi_planning/CostLearnerInterface.msg
// DO NOT EDIT!


#ifndef BWI_PLANNING_MESSAGE_COSTLEARNERINTERFACE_H
#define BWI_PLANNING_MESSAGE_COSTLEARNERINTERFACE_H

#include <ros/service_traits.h>


#include <bwi_planning/CostLearnerInterfaceRequest.h>
#include <bwi_planning/CostLearnerInterfaceResponse.h>


namespace bwi_planning
{

struct CostLearnerInterface
{

typedef CostLearnerInterfaceRequest Request;
typedef CostLearnerInterfaceResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct CostLearnerInterface
} // namespace bwi_planning


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::bwi_planning::CostLearnerInterface > {
  static const char* value()
  {
    return "61b3c456c1d44f026ba5c8c1f3d42630";
  }

  static const char* value(const ::bwi_planning::CostLearnerInterface&) { return value(); }
};

template<>
struct DataType< ::bwi_planning::CostLearnerInterface > {
  static const char* value()
  {
    return "bwi_planning/CostLearnerInterface";
  }

  static const char* value(const ::bwi_planning::CostLearnerInterface&) { return value(); }
};


// service_traits::MD5Sum< ::bwi_planning::CostLearnerInterfaceRequest> should match 
// service_traits::MD5Sum< ::bwi_planning::CostLearnerInterface > 
template<>
struct MD5Sum< ::bwi_planning::CostLearnerInterfaceRequest>
{
  static const char* value()
  {
    return MD5Sum< ::bwi_planning::CostLearnerInterface >::value();
  }
  static const char* value(const ::bwi_planning::CostLearnerInterfaceRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::bwi_planning::CostLearnerInterfaceRequest> should match 
// service_traits::DataType< ::bwi_planning::CostLearnerInterface > 
template<>
struct DataType< ::bwi_planning::CostLearnerInterfaceRequest>
{
  static const char* value()
  {
    return DataType< ::bwi_planning::CostLearnerInterface >::value();
  }
  static const char* value(const ::bwi_planning::CostLearnerInterfaceRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::bwi_planning::CostLearnerInterfaceResponse> should match 
// service_traits::MD5Sum< ::bwi_planning::CostLearnerInterface > 
template<>
struct MD5Sum< ::bwi_planning::CostLearnerInterfaceResponse>
{
  static const char* value()
  {
    return MD5Sum< ::bwi_planning::CostLearnerInterface >::value();
  }
  static const char* value(const ::bwi_planning::CostLearnerInterfaceResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::bwi_planning::CostLearnerInterfaceResponse> should match 
// service_traits::DataType< ::bwi_planning::CostLearnerInterface > 
template<>
struct DataType< ::bwi_planning::CostLearnerInterfaceResponse>
{
  static const char* value()
  {
    return DataType< ::bwi_planning::CostLearnerInterface >::value();
  }
  static const char* value(const ::bwi_planning::CostLearnerInterfaceResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // BWI_PLANNING_MESSAGE_COSTLEARNERINTERFACE_H