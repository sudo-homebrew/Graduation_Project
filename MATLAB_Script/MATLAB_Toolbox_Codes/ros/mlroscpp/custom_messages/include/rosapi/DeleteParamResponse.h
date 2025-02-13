// Generated by gencpp from file rosapi/DeleteParamResponse.msg
// DO NOT EDIT!


#ifndef ROSAPI_MESSAGE_DELETEPARAMRESPONSE_H
#define ROSAPI_MESSAGE_DELETEPARAMRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rosapi
{
template <class ContainerAllocator>
struct DeleteParamResponse_
{
  typedef DeleteParamResponse_<ContainerAllocator> Type;

  DeleteParamResponse_()
    {
    }
  DeleteParamResponse_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::rosapi::DeleteParamResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rosapi::DeleteParamResponse_<ContainerAllocator> const> ConstPtr;

}; // struct DeleteParamResponse_

typedef ::rosapi::DeleteParamResponse_<std::allocator<void> > DeleteParamResponse;

typedef boost::shared_ptr< ::rosapi::DeleteParamResponse > DeleteParamResponsePtr;
typedef boost::shared_ptr< ::rosapi::DeleteParamResponse const> DeleteParamResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rosapi::DeleteParamResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rosapi::DeleteParamResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace rosapi

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1352567/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'rosapi': ['/mathworks/home/pmurali/Documents/rosjava_messages/GEN-4-6/12/matlab_msg_gen_ros1/glnxa64/src/rosapi/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::rosapi::DeleteParamResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rosapi::DeleteParamResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rosapi::DeleteParamResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rosapi::DeleteParamResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rosapi::DeleteParamResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rosapi::DeleteParamResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rosapi::DeleteParamResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::rosapi::DeleteParamResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::rosapi::DeleteParamResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rosapi/DeleteParamResponse";
  }

  static const char* value(const ::rosapi::DeleteParamResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rosapi::DeleteParamResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::rosapi::DeleteParamResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rosapi::DeleteParamResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DeleteParamResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rosapi::DeleteParamResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::rosapi::DeleteParamResponse_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // ROSAPI_MESSAGE_DELETEPARAMRESPONSE_H
