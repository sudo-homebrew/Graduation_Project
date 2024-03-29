// Generated by gencpp from file capabilities/Capability.msg
// DO NOT EDIT!


#ifndef CAPABILITIES_MESSAGE_CAPABILITY_H
#define CAPABILITIES_MESSAGE_CAPABILITY_H


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
struct Capability_
{
  typedef Capability_<ContainerAllocator> Type;

  Capability_()
    : capability()
    , provider()  {
    }
  Capability_(const ContainerAllocator& _alloc)
    : capability(_alloc)
    , provider(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _capability_type;
  _capability_type capability;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _provider_type;
  _provider_type provider;





  typedef boost::shared_ptr< ::capabilities::Capability_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::capabilities::Capability_<ContainerAllocator> const> ConstPtr;

}; // struct Capability_

typedef ::capabilities::Capability_<std::allocator<void> > Capability;

typedef boost::shared_ptr< ::capabilities::Capability > CapabilityPtr;
typedef boost::shared_ptr< ::capabilities::Capability const> CapabilityConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::capabilities::Capability_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::capabilities::Capability_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::capabilities::Capability_<ContainerAllocator1> & lhs, const ::capabilities::Capability_<ContainerAllocator2> & rhs)
{
  return lhs.capability == rhs.capability &&
    lhs.provider == rhs.provider;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::capabilities::Capability_<ContainerAllocator1> & lhs, const ::capabilities::Capability_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace capabilities

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::capabilities::Capability_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::capabilities::Capability_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::capabilities::Capability_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::capabilities::Capability_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::capabilities::Capability_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::capabilities::Capability_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::capabilities::Capability_<ContainerAllocator> >
{
  static const char* value()
  {
    return "05f9dd41875315c324efdf915b0e33a9";
  }

  static const char* value(const ::capabilities::Capability_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x05f9dd41875315c3ULL;
  static const uint64_t static_value2 = 0x24efdf915b0e33a9ULL;
};

template<class ContainerAllocator>
struct DataType< ::capabilities::Capability_<ContainerAllocator> >
{
  static const char* value()
  {
    return "capabilities/Capability";
  }

  static const char* value(const ::capabilities::Capability_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::capabilities::Capability_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Capability\n"
"string capability\n"
"# Used provider\n"
"string provider\n"
;
  }

  static const char* value(const ::capabilities::Capability_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::capabilities::Capability_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.capability);
      stream.next(m.provider);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Capability_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::capabilities::Capability_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::capabilities::Capability_<ContainerAllocator>& v)
  {
    s << indent << "capability: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.capability);
    s << indent << "provider: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.provider);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CAPABILITIES_MESSAGE_CAPABILITY_H
