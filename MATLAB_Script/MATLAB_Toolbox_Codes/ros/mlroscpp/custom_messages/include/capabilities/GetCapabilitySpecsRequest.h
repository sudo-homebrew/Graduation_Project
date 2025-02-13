// Generated by gencpp from file capabilities/GetCapabilitySpecsRequest.msg
// DO NOT EDIT!


#ifndef CAPABILITIES_MESSAGE_GETCAPABILITYSPECSREQUEST_H
#define CAPABILITIES_MESSAGE_GETCAPABILITYSPECSREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace capabilities
{
template <class ContainerAllocator>
struct GetCapabilitySpecsRequest_
{
  typedef GetCapabilitySpecsRequest_<ContainerAllocator> Type;

  GetCapabilitySpecsRequest_()
    {
    }
  GetCapabilitySpecsRequest_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::capabilities::GetCapabilitySpecsRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::capabilities::GetCapabilitySpecsRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GetCapabilitySpecsRequest_

typedef ::capabilities::GetCapabilitySpecsRequest_<std::allocator<void> > GetCapabilitySpecsRequest;

typedef boost::shared_ptr< ::capabilities::GetCapabilitySpecsRequest > GetCapabilitySpecsRequestPtr;
typedef boost::shared_ptr< ::capabilities::GetCapabilitySpecsRequest const> GetCapabilitySpecsRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::capabilities::GetCapabilitySpecsRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::capabilities::GetCapabilitySpecsRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace capabilities

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::capabilities::GetCapabilitySpecsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::capabilities::GetCapabilitySpecsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::capabilities::GetCapabilitySpecsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::capabilities::GetCapabilitySpecsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::capabilities::GetCapabilitySpecsRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::capabilities::GetCapabilitySpecsRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::capabilities::GetCapabilitySpecsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::capabilities::GetCapabilitySpecsRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::capabilities::GetCapabilitySpecsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "capabilities/GetCapabilitySpecsRequest";
  }

  static const char* value(const ::capabilities::GetCapabilitySpecsRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::capabilities::GetCapabilitySpecsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::capabilities::GetCapabilitySpecsRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::capabilities::GetCapabilitySpecsRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetCapabilitySpecsRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::capabilities::GetCapabilitySpecsRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::capabilities::GetCapabilitySpecsRequest_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // CAPABILITIES_MESSAGE_GETCAPABILITYSPECSREQUEST_H
