// Generated by gencpp from file saphari_msgs/PerceiveEquipmentRequest.msg
// DO NOT EDIT!


#ifndef SAPHARI_MSGS_MESSAGE_PERCEIVEEQUIPMENTREQUEST_H
#define SAPHARI_MSGS_MESSAGE_PERCEIVEEQUIPMENTREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace saphari_msgs
{
template <class ContainerAllocator>
struct PerceiveEquipmentRequest_
{
  typedef PerceiveEquipmentRequest_<ContainerAllocator> Type;

  PerceiveEquipmentRequest_()
    : request()  {
    }
  PerceiveEquipmentRequest_(const ContainerAllocator& _alloc)
    : request(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _request_type;
  _request_type request;





  typedef boost::shared_ptr< ::saphari_msgs::PerceiveEquipmentRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::saphari_msgs::PerceiveEquipmentRequest_<ContainerAllocator> const> ConstPtr;

}; // struct PerceiveEquipmentRequest_

typedef ::saphari_msgs::PerceiveEquipmentRequest_<std::allocator<void> > PerceiveEquipmentRequest;

typedef boost::shared_ptr< ::saphari_msgs::PerceiveEquipmentRequest > PerceiveEquipmentRequestPtr;
typedef boost::shared_ptr< ::saphari_msgs::PerceiveEquipmentRequest const> PerceiveEquipmentRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::saphari_msgs::PerceiveEquipmentRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::saphari_msgs::PerceiveEquipmentRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace saphari_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/local-ssd1/All_Custom_Msgs/packages_in_cluster/matlab_msg_gen_ros1/glnxa64/src/geometry_msgs/msg'], 'actionlib': ['/mathworks/devel/sbs/31/hakakarl.Brobot.j1368129/matlab/sys/ros1/glnxa64/ros1/share/actionlib/cmake/../msg'], 'saphari_msgs': ['/local-ssd1/All_Custom_Msgs/packages_in_cluster/matlab_msg_gen_ros1/glnxa64/src/saphari_msgs/msg'], 'std_msgs': ['/mathworks/devel/sbs/31/hakakarl.Brobot.j1368129/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg', '/mathworks/devel/sbs/31/hakakarl.Brobot.j1368129/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/mathworks/devel/sbs/31/hakakarl.Brobot.j1368129/matlab/sys/ros1/glnxa64/ros1/share/actionlib_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::saphari_msgs::PerceiveEquipmentRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::saphari_msgs::PerceiveEquipmentRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::saphari_msgs::PerceiveEquipmentRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::saphari_msgs::PerceiveEquipmentRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::saphari_msgs::PerceiveEquipmentRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::saphari_msgs::PerceiveEquipmentRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::saphari_msgs::PerceiveEquipmentRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9b13f31f7a0a36901919f7ec0d9f40d4";
  }

  static const char* value(const ::saphari_msgs::PerceiveEquipmentRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9b13f31f7a0a3690ULL;
  static const uint64_t static_value2 = 0x1919f7ec0d9f40d4ULL;
};

template<class ContainerAllocator>
struct DataType< ::saphari_msgs::PerceiveEquipmentRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "saphari_msgs/PerceiveEquipmentRequest";
  }

  static const char* value(const ::saphari_msgs::PerceiveEquipmentRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::saphari_msgs::PerceiveEquipmentRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string request\n"
;
  }

  static const char* value(const ::saphari_msgs::PerceiveEquipmentRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::saphari_msgs::PerceiveEquipmentRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.request);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PerceiveEquipmentRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::saphari_msgs::PerceiveEquipmentRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::saphari_msgs::PerceiveEquipmentRequest_<ContainerAllocator>& v)
  {
    s << indent << "request: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.request);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SAPHARI_MSGS_MESSAGE_PERCEIVEEQUIPMENTREQUEST_H