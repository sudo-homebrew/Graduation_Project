// Generated by gencpp from file mongodb_store/SetParamRequest.msg
// DO NOT EDIT!


#ifndef MONGODB_STORE_MESSAGE_SETPARAMREQUEST_H
#define MONGODB_STORE_MESSAGE_SETPARAMREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace mongodb_store
{
template <class ContainerAllocator>
struct SetParamRequest_
{
  typedef SetParamRequest_<ContainerAllocator> Type;

  SetParamRequest_()
    : param()  {
    }
  SetParamRequest_(const ContainerAllocator& _alloc)
    : param(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _param_type;
  _param_type param;





  typedef boost::shared_ptr< ::mongodb_store::SetParamRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mongodb_store::SetParamRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetParamRequest_

typedef ::mongodb_store::SetParamRequest_<std::allocator<void> > SetParamRequest;

typedef boost::shared_ptr< ::mongodb_store::SetParamRequest > SetParamRequestPtr;
typedef boost::shared_ptr< ::mongodb_store::SetParamRequest const> SetParamRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mongodb_store::SetParamRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mongodb_store::SetParamRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace mongodb_store

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1352567/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::mongodb_store::SetParamRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mongodb_store::SetParamRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mongodb_store::SetParamRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mongodb_store::SetParamRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mongodb_store::SetParamRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mongodb_store::SetParamRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mongodb_store::SetParamRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "eb04b7504512676dca105ab8842899a4";
  }

  static const char* value(const ::mongodb_store::SetParamRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xeb04b7504512676dULL;
  static const uint64_t static_value2 = 0xca105ab8842899a4ULL;
};

template<class ContainerAllocator>
struct DataType< ::mongodb_store::SetParamRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mongodb_store/SetParamRequest";
  }

  static const char* value(const ::mongodb_store::SetParamRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mongodb_store::SetParamRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string param\n"
;
  }

  static const char* value(const ::mongodb_store::SetParamRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mongodb_store::SetParamRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.param);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetParamRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mongodb_store::SetParamRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mongodb_store::SetParamRequest_<ContainerAllocator>& v)
  {
    s << indent << "param: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.param);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MONGODB_STORE_MESSAGE_SETPARAMREQUEST_H