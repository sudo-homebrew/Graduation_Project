// Generated by gencpp from file nao_msgs/BlinkFeedback.msg
// DO NOT EDIT!


#ifndef NAO_MSGS_MESSAGE_BLINKFEEDBACK_H
#define NAO_MSGS_MESSAGE_BLINKFEEDBACK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/ColorRGBA.h>

namespace nao_msgs
{
template <class ContainerAllocator>
struct BlinkFeedback_
{
  typedef BlinkFeedback_<ContainerAllocator> Type;

  BlinkFeedback_()
    : last_color()  {
    }
  BlinkFeedback_(const ContainerAllocator& _alloc)
    : last_color(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::ColorRGBA_<ContainerAllocator>  _last_color_type;
  _last_color_type last_color;





  typedef boost::shared_ptr< ::nao_msgs::BlinkFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nao_msgs::BlinkFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct BlinkFeedback_

typedef ::nao_msgs::BlinkFeedback_<std::allocator<void> > BlinkFeedback;

typedef boost::shared_ptr< ::nao_msgs::BlinkFeedback > BlinkFeedbackPtr;
typedef boost::shared_ptr< ::nao_msgs::BlinkFeedback const> BlinkFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::nao_msgs::BlinkFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::nao_msgs::BlinkFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace nao_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/local-ssd1/All_Custom_Msgs/packages_in_cluster/matlab_msg_gen_ros1/glnxa64/src/nav_msgs/msg', '/local-ssd1/All_Custom_Msgs/packages_in_cluster/matlab_msg_gen_ros1/glnxa64/devel/share/nav_msgs/msg'], 'nao_msgs': ['/local-ssd1/All_Custom_Msgs/packages_in_cluster/matlab_msg_gen_ros1/glnxa64/src/nao_msgs/msg', '/local-ssd1/All_Custom_Msgs/packages_in_cluster/matlab_msg_gen_ros1/glnxa64/devel/share/nao_msgs/msg'], 'sensor_msgs': ['/local-ssd1/All_Custom_Msgs/packages_in_cluster/matlab_msg_gen_ros1/glnxa64/src/sensor_msgs/msg'], 'actionlib': ['/mathworks/devel/sbs/31/hakakarl.Brobot.j1368129/matlab/sys/ros1/glnxa64/ros1/share/actionlib/cmake/../msg'], 'trajectory_msgs': ['/local-ssd1/All_Custom_Msgs/packages_in_cluster/matlab_msg_gen_ros1/glnxa64/src/trajectory_msgs/msg'], 'std_msgs': ['/mathworks/devel/sbs/31/hakakarl.Brobot.j1368129/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg', '/mathworks/devel/sbs/31/hakakarl.Brobot.j1368129/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/local-ssd1/All_Custom_Msgs/packages_in_cluster/matlab_msg_gen_ros1/glnxa64/src/geometry_msgs/msg'], 'actionlib_msgs': ['/mathworks/devel/sbs/31/hakakarl.Brobot.j1368129/matlab/sys/ros1/glnxa64/ros1/share/actionlib_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::nao_msgs::BlinkFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::nao_msgs::BlinkFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::nao_msgs::BlinkFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::nao_msgs::BlinkFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::nao_msgs::BlinkFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::nao_msgs::BlinkFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::nao_msgs::BlinkFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6f1f94fb3eb06412264f6e0c5e72cfab";
  }

  static const char* value(const ::nao_msgs::BlinkFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6f1f94fb3eb06412ULL;
  static const uint64_t static_value2 = 0x264f6e0c5e72cfabULL;
};

template<class ContainerAllocator>
struct DataType< ::nao_msgs::BlinkFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "nao_msgs/BlinkFeedback";
  }

  static const char* value(const ::nao_msgs::BlinkFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::nao_msgs::BlinkFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"std_msgs/ColorRGBA last_color\n"
"\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/ColorRGBA\n"
"float32 r\n"
"float32 g\n"
"float32 b\n"
"float32 a\n"
;
  }

  static const char* value(const ::nao_msgs::BlinkFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::nao_msgs::BlinkFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.last_color);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct BlinkFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::nao_msgs::BlinkFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::nao_msgs::BlinkFeedback_<ContainerAllocator>& v)
  {
    s << indent << "last_color: ";
    s << std::endl;
    Printer< ::std_msgs::ColorRGBA_<ContainerAllocator> >::stream(s, indent + "  ", v.last_color);
  }
};

} // namespace message_operations
} // namespace ros

#endif // NAO_MSGS_MESSAGE_BLINKFEEDBACK_H