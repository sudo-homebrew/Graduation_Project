// Generated by gencpp from file capabilities/GetRunningCapabilitiesResponse.msg
// DO NOT EDIT!


#ifndef CAPABILITIES_MESSAGE_GETRUNNINGCAPABILITIESRESPONSE_H
#define CAPABILITIES_MESSAGE_GETRUNNINGCAPABILITIESRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <capabilities/RunningCapability.h>

namespace capabilities
{
template <class ContainerAllocator>
struct GetRunningCapabilitiesResponse_
{
  typedef GetRunningCapabilitiesResponse_<ContainerAllocator> Type;

  GetRunningCapabilitiesResponse_()
    : running_capabilities()  {
    }
  GetRunningCapabilitiesResponse_(const ContainerAllocator& _alloc)
    : running_capabilities(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::capabilities::RunningCapability_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::capabilities::RunningCapability_<ContainerAllocator> >::other >  _running_capabilities_type;
  _running_capabilities_type running_capabilities;





  typedef boost::shared_ptr< ::capabilities::GetRunningCapabilitiesResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::capabilities::GetRunningCapabilitiesResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetRunningCapabilitiesResponse_

typedef ::capabilities::GetRunningCapabilitiesResponse_<std::allocator<void> > GetRunningCapabilitiesResponse;

typedef boost::shared_ptr< ::capabilities::GetRunningCapabilitiesResponse > GetRunningCapabilitiesResponsePtr;
typedef boost::shared_ptr< ::capabilities::GetRunningCapabilitiesResponse const> GetRunningCapabilitiesResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::capabilities::GetRunningCapabilitiesResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::capabilities::GetRunningCapabilitiesResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::capabilities::GetRunningCapabilitiesResponse_<ContainerAllocator1> & lhs, const ::capabilities::GetRunningCapabilitiesResponse_<ContainerAllocator2> & rhs)
{
  return lhs.running_capabilities == rhs.running_capabilities;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::capabilities::GetRunningCapabilitiesResponse_<ContainerAllocator1> & lhs, const ::capabilities::GetRunningCapabilitiesResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace capabilities

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::capabilities::GetRunningCapabilitiesResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::capabilities::GetRunningCapabilitiesResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::capabilities::GetRunningCapabilitiesResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::capabilities::GetRunningCapabilitiesResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::capabilities::GetRunningCapabilitiesResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::capabilities::GetRunningCapabilitiesResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::capabilities::GetRunningCapabilitiesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e8e5cf68c34711ffaa719728f34ea7a3";
  }

  static const char* value(const ::capabilities::GetRunningCapabilitiesResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe8e5cf68c34711ffULL;
  static const uint64_t static_value2 = 0xaa719728f34ea7a3ULL;
};

template<class ContainerAllocator>
struct DataType< ::capabilities::GetRunningCapabilitiesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "capabilities/GetRunningCapabilitiesResponse";
  }

  static const char* value(const ::capabilities::GetRunningCapabilitiesResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::capabilities::GetRunningCapabilitiesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "RunningCapability[] running_capabilities\n"
"\n"
"\n"
"================================================================================\n"
"MSG: capabilities/RunningCapability\n"
"# Name and provider of this running capability\n"
"Capability capability\n"
"# Capabilities which depend on this one\n"
"Capability[] dependent_capabilities\n"
"# Message stating what started this capability\n"
"string started_by\n"
"# Process ID of the running provider\n"
"int32 pid\n"
"\n"
"================================================================================\n"
"MSG: capabilities/Capability\n"
"# Capability\n"
"string capability\n"
"# Used provider\n"
"string provider\n"
;
  }

  static const char* value(const ::capabilities::GetRunningCapabilitiesResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::capabilities::GetRunningCapabilitiesResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.running_capabilities);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetRunningCapabilitiesResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::capabilities::GetRunningCapabilitiesResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::capabilities::GetRunningCapabilitiesResponse_<ContainerAllocator>& v)
  {
    s << indent << "running_capabilities[]" << std::endl;
    for (size_t i = 0; i < v.running_capabilities.size(); ++i)
    {
      s << indent << "  running_capabilities[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::capabilities::RunningCapability_<ContainerAllocator> >::stream(s, indent + "    ", v.running_capabilities[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // CAPABILITIES_MESSAGE_GETRUNNINGCAPABILITIESRESPONSE_H