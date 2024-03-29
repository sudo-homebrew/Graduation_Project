// Generated by gencpp from file nao_msgs/SetTransformRequest.msg
// DO NOT EDIT!


#ifndef NAO_MSGS_MESSAGE_SETTRANSFORMREQUEST_H
#define NAO_MSGS_MESSAGE_SETTRANSFORMREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Transform.h>

namespace nao_msgs
{
template <class ContainerAllocator>
struct SetTransformRequest_
{
  typedef SetTransformRequest_<ContainerAllocator> Type;

  SetTransformRequest_()
    : offset()  {
    }
  SetTransformRequest_(const ContainerAllocator& _alloc)
    : offset(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Transform_<ContainerAllocator>  _offset_type;
  _offset_type offset;





  typedef boost::shared_ptr< ::nao_msgs::SetTransformRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nao_msgs::SetTransformRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetTransformRequest_

typedef ::nao_msgs::SetTransformRequest_<std::allocator<void> > SetTransformRequest;

typedef boost::shared_ptr< ::nao_msgs::SetTransformRequest > SetTransformRequestPtr;
typedef boost::shared_ptr< ::nao_msgs::SetTransformRequest const> SetTransformRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::nao_msgs::SetTransformRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::nao_msgs::SetTransformRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace nao_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1339814/matlab/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg'], 'nao_msgs': ['/mathworks/home/pmurali/Documents/MNO/matlab_msg_gen_ros1/glnxa64/src/nao_msgs/msg'], 'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1339814/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg', '/mathworks/devel/sbs/31/pmurali.Brobot.j1339814/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::nao_msgs::SetTransformRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::nao_msgs::SetTransformRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::nao_msgs::SetTransformRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::nao_msgs::SetTransformRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::nao_msgs::SetTransformRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::nao_msgs::SetTransformRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::nao_msgs::SetTransformRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "67035ddf415a9bb64191f0e45b060e35";
  }

  static const char* value(const ::nao_msgs::SetTransformRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x67035ddf415a9bb6ULL;
  static const uint64_t static_value2 = 0x4191f0e45b060e35ULL;
};

template<class ContainerAllocator>
struct DataType< ::nao_msgs::SetTransformRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "nao_msgs/SetTransformRequest";
  }

  static const char* value(const ::nao_msgs::SetTransformRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::nao_msgs::SetTransformRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"geometry_msgs/Transform offset\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Transform\n"
"# This represents the transform between two coordinate frames in free space.\n"
"\n"
"Vector3 translation\n"
"Quaternion rotation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::nao_msgs::SetTransformRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::nao_msgs::SetTransformRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.offset);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetTransformRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::nao_msgs::SetTransformRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::nao_msgs::SetTransformRequest_<ContainerAllocator>& v)
  {
    s << indent << "offset: ";
    s << std::endl;
    Printer< ::geometry_msgs::Transform_<ContainerAllocator> >::stream(s, indent + "  ", v.offset);
  }
};

} // namespace message_operations
} // namespace ros

#endif // NAO_MSGS_MESSAGE_SETTRANSFORMREQUEST_H
