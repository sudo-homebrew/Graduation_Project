// Generated by gencpp from file jsk_pcl_ros/CheckCircleResponse.msg
// DO NOT EDIT!


#ifndef JSK_PCL_ROS_MESSAGE_CHECKCIRCLERESPONSE_H
#define JSK_PCL_ROS_MESSAGE_CHECKCIRCLERESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace jsk_pcl_ros
{
template <class ContainerAllocator>
struct CheckCircleResponse_
{
  typedef CheckCircleResponse_<ContainerAllocator> Type;

  CheckCircleResponse_()
    : clicked(false)
    , index(0)
    , msg()  {
    }
  CheckCircleResponse_(const ContainerAllocator& _alloc)
    : clicked(false)
    , index(0)
    , msg(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _clicked_type;
  _clicked_type clicked;

   typedef int32_t _index_type;
  _index_type index;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _msg_type;
  _msg_type msg;





  typedef boost::shared_ptr< ::jsk_pcl_ros::CheckCircleResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_pcl_ros::CheckCircleResponse_<ContainerAllocator> const> ConstPtr;

}; // struct CheckCircleResponse_

typedef ::jsk_pcl_ros::CheckCircleResponse_<std::allocator<void> > CheckCircleResponse;

typedef boost::shared_ptr< ::jsk_pcl_ros::CheckCircleResponse > CheckCircleResponsePtr;
typedef boost::shared_ptr< ::jsk_pcl_ros::CheckCircleResponse const> CheckCircleResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_pcl_ros::CheckCircleResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_pcl_ros::CheckCircleResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace jsk_pcl_ros

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1339814/matlab/sys/ros1/glnxa64/ros1/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1339814/matlab/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg'], 'jsk_pcl_ros': ['/mathworks/home/pmurali/Documents/JKL/matlab_msg_gen_ros1/glnxa64/src/jsk_pcl_ros/msg'], 'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1339814/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg', '/mathworks/devel/sbs/31/pmurali.Brobot.j1339814/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'pcl_msgs': ['/mathworks/home/pmurali/Documents/JKL/matlab_msg_gen_ros1/glnxa64/src/pcl_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::jsk_pcl_ros::CheckCircleResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_pcl_ros::CheckCircleResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_pcl_ros::CheckCircleResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_pcl_ros::CheckCircleResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_pcl_ros::CheckCircleResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_pcl_ros::CheckCircleResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_pcl_ros::CheckCircleResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "94ed41c732187b6ea58431df72ab10b2";
  }

  static const char* value(const ::jsk_pcl_ros::CheckCircleResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x94ed41c732187b6eULL;
  static const uint64_t static_value2 = 0xa58431df72ab10b2ULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_pcl_ros::CheckCircleResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_pcl_ros/CheckCircleResponse";
  }

  static const char* value(const ::jsk_pcl_ros::CheckCircleResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_pcl_ros::CheckCircleResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool clicked\n"
"int32 index\n"
"string msg\n"
"\n"
;
  }

  static const char* value(const ::jsk_pcl_ros::CheckCircleResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_pcl_ros::CheckCircleResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.clicked);
      stream.next(m.index);
      stream.next(m.msg);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CheckCircleResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_pcl_ros::CheckCircleResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_pcl_ros::CheckCircleResponse_<ContainerAllocator>& v)
  {
    s << indent << "clicked: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.clicked);
    s << indent << "index: ";
    Printer<int32_t>::stream(s, indent + "  ", v.index);
    s << indent << "msg: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.msg);
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_PCL_ROS_MESSAGE_CHECKCIRCLERESPONSE_H