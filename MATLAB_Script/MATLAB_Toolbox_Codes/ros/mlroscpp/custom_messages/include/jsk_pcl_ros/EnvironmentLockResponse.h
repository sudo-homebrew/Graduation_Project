// Generated by gencpp from file jsk_pcl_ros/EnvironmentLockResponse.msg
// DO NOT EDIT!


#ifndef JSK_PCL_ROS_MESSAGE_ENVIRONMENTLOCKRESPONSE_H
#define JSK_PCL_ROS_MESSAGE_ENVIRONMENTLOCKRESPONSE_H


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
struct EnvironmentLockResponse_
{
  typedef EnvironmentLockResponse_<ContainerAllocator> Type;

  EnvironmentLockResponse_()
    : environment_id(0)  {
    }
  EnvironmentLockResponse_(const ContainerAllocator& _alloc)
    : environment_id(0)  {
  (void)_alloc;
    }



   typedef uint32_t _environment_id_type;
  _environment_id_type environment_id;





  typedef boost::shared_ptr< ::jsk_pcl_ros::EnvironmentLockResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_pcl_ros::EnvironmentLockResponse_<ContainerAllocator> const> ConstPtr;

}; // struct EnvironmentLockResponse_

typedef ::jsk_pcl_ros::EnvironmentLockResponse_<std::allocator<void> > EnvironmentLockResponse;

typedef boost::shared_ptr< ::jsk_pcl_ros::EnvironmentLockResponse > EnvironmentLockResponsePtr;
typedef boost::shared_ptr< ::jsk_pcl_ros::EnvironmentLockResponse const> EnvironmentLockResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_pcl_ros::EnvironmentLockResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_pcl_ros::EnvironmentLockResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace jsk_pcl_ros

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1339814/matlab/sys/ros1/glnxa64/ros1/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1339814/matlab/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg'], 'jsk_pcl_ros': ['/mathworks/home/pmurali/Documents/JKL/matlab_msg_gen_ros1/glnxa64/src/jsk_pcl_ros/msg'], 'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1339814/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg', '/mathworks/devel/sbs/31/pmurali.Brobot.j1339814/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'pcl_msgs': ['/mathworks/home/pmurali/Documents/JKL/matlab_msg_gen_ros1/glnxa64/src/pcl_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::jsk_pcl_ros::EnvironmentLockResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_pcl_ros::EnvironmentLockResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_pcl_ros::EnvironmentLockResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_pcl_ros::EnvironmentLockResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_pcl_ros::EnvironmentLockResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_pcl_ros::EnvironmentLockResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_pcl_ros::EnvironmentLockResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "109afc0d3bd22aa461d45c8ef5ab6d75";
  }

  static const char* value(const ::jsk_pcl_ros::EnvironmentLockResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x109afc0d3bd22aa4ULL;
  static const uint64_t static_value2 = 0x61d45c8ef5ab6d75ULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_pcl_ros::EnvironmentLockResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_pcl_ros/EnvironmentLockResponse";
  }

  static const char* value(const ::jsk_pcl_ros::EnvironmentLockResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_pcl_ros::EnvironmentLockResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint32 environment_id\n"
"\n"
;
  }

  static const char* value(const ::jsk_pcl_ros::EnvironmentLockResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_pcl_ros::EnvironmentLockResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.environment_id);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct EnvironmentLockResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_pcl_ros::EnvironmentLockResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_pcl_ros::EnvironmentLockResponse_<ContainerAllocator>& v)
  {
    s << indent << "environment_id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.environment_id);
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_PCL_ROS_MESSAGE_ENVIRONMENTLOCKRESPONSE_H