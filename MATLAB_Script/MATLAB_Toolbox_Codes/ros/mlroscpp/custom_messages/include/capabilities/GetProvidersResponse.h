// Generated by gencpp from file capabilities/GetProvidersResponse.msg
// DO NOT EDIT!


#ifndef CAPABILITIES_MESSAGE_GETPROVIDERSRESPONSE_H
#define CAPABILITIES_MESSAGE_GETPROVIDERSRESPONSE_H


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
struct GetProvidersResponse_
{
  typedef GetProvidersResponse_<ContainerAllocator> Type;

  GetProvidersResponse_()
    : providers()
    , default_provider()  {
    }
  GetProvidersResponse_(const ContainerAllocator& _alloc)
    : providers(_alloc)
    , default_provider(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _providers_type;
  _providers_type providers;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _default_provider_type;
  _default_provider_type default_provider;





  typedef boost::shared_ptr< ::capabilities::GetProvidersResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::capabilities::GetProvidersResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetProvidersResponse_

typedef ::capabilities::GetProvidersResponse_<std::allocator<void> > GetProvidersResponse;

typedef boost::shared_ptr< ::capabilities::GetProvidersResponse > GetProvidersResponsePtr;
typedef boost::shared_ptr< ::capabilities::GetProvidersResponse const> GetProvidersResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::capabilities::GetProvidersResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::capabilities::GetProvidersResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::capabilities::GetProvidersResponse_<ContainerAllocator1> & lhs, const ::capabilities::GetProvidersResponse_<ContainerAllocator2> & rhs)
{
  return lhs.providers == rhs.providers &&
    lhs.default_provider == rhs.default_provider;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::capabilities::GetProvidersResponse_<ContainerAllocator1> & lhs, const ::capabilities::GetProvidersResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace capabilities

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::capabilities::GetProvidersResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::capabilities::GetProvidersResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::capabilities::GetProvidersResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::capabilities::GetProvidersResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::capabilities::GetProvidersResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::capabilities::GetProvidersResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::capabilities::GetProvidersResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f07bf2dab3c1c90f7df32f0732047bbd";
  }

  static const char* value(const ::capabilities::GetProvidersResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf07bf2dab3c1c90fULL;
  static const uint64_t static_value2 = 0x7df32f0732047bbdULL;
};

template<class ContainerAllocator>
struct DataType< ::capabilities::GetProvidersResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "capabilities/GetProvidersResponse";
  }

  static const char* value(const ::capabilities::GetProvidersResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::capabilities::GetProvidersResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string[] providers\n"
"string default_provider\n"
"\n"
;
  }

  static const char* value(const ::capabilities::GetProvidersResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::capabilities::GetProvidersResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.providers);
      stream.next(m.default_provider);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetProvidersResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::capabilities::GetProvidersResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::capabilities::GetProvidersResponse_<ContainerAllocator>& v)
  {
    s << indent << "providers[]" << std::endl;
    for (size_t i = 0; i < v.providers.size(); ++i)
    {
      s << indent << "  providers[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.providers[i]);
    }
    s << indent << "default_provider: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.default_provider);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CAPABILITIES_MESSAGE_GETPROVIDERSRESPONSE_H