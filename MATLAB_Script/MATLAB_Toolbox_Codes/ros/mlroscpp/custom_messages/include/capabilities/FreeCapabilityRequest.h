// Generated by gencpp from file capabilities/FreeCapabilityRequest.msg
// DO NOT EDIT!


#ifndef CAPABILITIES_MESSAGE_FREECAPABILITYREQUEST_H
#define CAPABILITIES_MESSAGE_FREECAPABILITYREQUEST_H


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
struct FreeCapabilityRequest_
{
  typedef FreeCapabilityRequest_<ContainerAllocator> Type;

  FreeCapabilityRequest_()
    : capability()
    , bond_id()  {
    }
  FreeCapabilityRequest_(const ContainerAllocator& _alloc)
    : capability(_alloc)
    , bond_id(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _capability_type;
  _capability_type capability;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _bond_id_type;
  _bond_id_type bond_id;





  typedef boost::shared_ptr< ::capabilities::FreeCapabilityRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::capabilities::FreeCapabilityRequest_<ContainerAllocator> const> ConstPtr;

}; // struct FreeCapabilityRequest_

typedef ::capabilities::FreeCapabilityRequest_<std::allocator<void> > FreeCapabilityRequest;

typedef boost::shared_ptr< ::capabilities::FreeCapabilityRequest > FreeCapabilityRequestPtr;
typedef boost::shared_ptr< ::capabilities::FreeCapabilityRequest const> FreeCapabilityRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::capabilities::FreeCapabilityRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::capabilities::FreeCapabilityRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::capabilities::FreeCapabilityRequest_<ContainerAllocator1> & lhs, const ::capabilities::FreeCapabilityRequest_<ContainerAllocator2> & rhs)
{
  return lhs.capability == rhs.capability &&
    lhs.bond_id == rhs.bond_id;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::capabilities::FreeCapabilityRequest_<ContainerAllocator1> & lhs, const ::capabilities::FreeCapabilityRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace capabilities

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::capabilities::FreeCapabilityRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::capabilities::FreeCapabilityRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::capabilities::FreeCapabilityRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::capabilities::FreeCapabilityRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::capabilities::FreeCapabilityRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::capabilities::FreeCapabilityRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::capabilities::FreeCapabilityRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a428ffea976541636efe63c8605fd3ec";
  }

  static const char* value(const ::capabilities::FreeCapabilityRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa428ffea97654163ULL;
  static const uint64_t static_value2 = 0x6efe63c8605fd3ecULL;
};

template<class ContainerAllocator>
struct DataType< ::capabilities::FreeCapabilityRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "capabilities/FreeCapabilityRequest";
  }

  static const char* value(const ::capabilities::FreeCapabilityRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::capabilities::FreeCapabilityRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string capability\n"
"string bond_id\n"
;
  }

  static const char* value(const ::capabilities::FreeCapabilityRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::capabilities::FreeCapabilityRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.capability);
      stream.next(m.bond_id);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FreeCapabilityRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::capabilities::FreeCapabilityRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::capabilities::FreeCapabilityRequest_<ContainerAllocator>& v)
  {
    s << indent << "capability: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.capability);
    s << indent << "bond_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.bond_id);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CAPABILITIES_MESSAGE_FREECAPABILITYREQUEST_H