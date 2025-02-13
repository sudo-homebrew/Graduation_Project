// Generated by gencpp from file jsk_pcl_ros/PolygonOnEnvironmentRequest.msg
// DO NOT EDIT!


#ifndef JSK_PCL_ROS_MESSAGE_POLYGONONENVIRONMENTREQUEST_H
#define JSK_PCL_ROS_MESSAGE_POLYGONONENVIRONMENTREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/PolygonStamped.h>

namespace jsk_pcl_ros
{
template <class ContainerAllocator>
struct PolygonOnEnvironmentRequest_
{
  typedef PolygonOnEnvironmentRequest_<ContainerAllocator> Type;

  PolygonOnEnvironmentRequest_()
    : environment_id(0)
    , plane_index(0)
    , polygon()  {
    }
  PolygonOnEnvironmentRequest_(const ContainerAllocator& _alloc)
    : environment_id(0)
    , plane_index(0)
    , polygon(_alloc)  {
  (void)_alloc;
    }



   typedef uint32_t _environment_id_type;
  _environment_id_type environment_id;

   typedef uint32_t _plane_index_type;
  _plane_index_type plane_index;

   typedef  ::geometry_msgs::PolygonStamped_<ContainerAllocator>  _polygon_type;
  _polygon_type polygon;





  typedef boost::shared_ptr< ::jsk_pcl_ros::PolygonOnEnvironmentRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_pcl_ros::PolygonOnEnvironmentRequest_<ContainerAllocator> const> ConstPtr;

}; // struct PolygonOnEnvironmentRequest_

typedef ::jsk_pcl_ros::PolygonOnEnvironmentRequest_<std::allocator<void> > PolygonOnEnvironmentRequest;

typedef boost::shared_ptr< ::jsk_pcl_ros::PolygonOnEnvironmentRequest > PolygonOnEnvironmentRequestPtr;
typedef boost::shared_ptr< ::jsk_pcl_ros::PolygonOnEnvironmentRequest const> PolygonOnEnvironmentRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_pcl_ros::PolygonOnEnvironmentRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_pcl_ros::PolygonOnEnvironmentRequest_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::jsk_pcl_ros::PolygonOnEnvironmentRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_pcl_ros::PolygonOnEnvironmentRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_pcl_ros::PolygonOnEnvironmentRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_pcl_ros::PolygonOnEnvironmentRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_pcl_ros::PolygonOnEnvironmentRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_pcl_ros::PolygonOnEnvironmentRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_pcl_ros::PolygonOnEnvironmentRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5c876f97015e6a599aa3c09455882c02";
  }

  static const char* value(const ::jsk_pcl_ros::PolygonOnEnvironmentRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5c876f97015e6a59ULL;
  static const uint64_t static_value2 = 0x9aa3c09455882c02ULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_pcl_ros::PolygonOnEnvironmentRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_pcl_ros/PolygonOnEnvironmentRequest";
  }

  static const char* value(const ::jsk_pcl_ros::PolygonOnEnvironmentRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_pcl_ros::PolygonOnEnvironmentRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint32 environment_id\n"
"uint32 plane_index\n"
"geometry_msgs/PolygonStamped polygon\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/PolygonStamped\n"
"# This represents a Polygon with reference coordinate frame and timestamp\n"
"Header header\n"
"Polygon polygon\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Polygon\n"
"#A specification of a polygon where the first and last points are assumed to be connected\n"
"Point32[] points\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point32\n"
"# This contains the position of a point in free space(with 32 bits of precision).\n"
"# It is recommeded to use Point wherever possible instead of Point32.  \n"
"# \n"
"# This recommendation is to promote interoperability.  \n"
"#\n"
"# This message is designed to take up less space when sending\n"
"# lots of points at once, as in the case of a PointCloud.  \n"
"\n"
"float32 x\n"
"float32 y\n"
"float32 z\n"
;
  }

  static const char* value(const ::jsk_pcl_ros::PolygonOnEnvironmentRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_pcl_ros::PolygonOnEnvironmentRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.environment_id);
      stream.next(m.plane_index);
      stream.next(m.polygon);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PolygonOnEnvironmentRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_pcl_ros::PolygonOnEnvironmentRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_pcl_ros::PolygonOnEnvironmentRequest_<ContainerAllocator>& v)
  {
    s << indent << "environment_id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.environment_id);
    s << indent << "plane_index: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.plane_index);
    s << indent << "polygon: ";
    s << std::endl;
    Printer< ::geometry_msgs::PolygonStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.polygon);
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_PCL_ROS_MESSAGE_POLYGONONENVIRONMENTREQUEST_H
