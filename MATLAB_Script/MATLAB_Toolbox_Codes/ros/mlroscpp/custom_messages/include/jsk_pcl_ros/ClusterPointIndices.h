// Generated by gencpp from file jsk_pcl_ros/ClusterPointIndices.msg
// DO NOT EDIT!


#ifndef JSK_PCL_ROS_MESSAGE_CLUSTERPOINTINDICES_H
#define JSK_PCL_ROS_MESSAGE_CLUSTERPOINTINDICES_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <pcl_msgs/PointIndices.h>

namespace jsk_pcl_ros
{
template <class ContainerAllocator>
struct ClusterPointIndices_
{
  typedef ClusterPointIndices_<ContainerAllocator> Type;

  ClusterPointIndices_()
    : header()
    , cluster_indices()  {
    }
  ClusterPointIndices_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , cluster_indices(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::pcl_msgs::PointIndices_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::pcl_msgs::PointIndices_<ContainerAllocator> >::other >  _cluster_indices_type;
  _cluster_indices_type cluster_indices;





  typedef boost::shared_ptr< ::jsk_pcl_ros::ClusterPointIndices_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_pcl_ros::ClusterPointIndices_<ContainerAllocator> const> ConstPtr;

}; // struct ClusterPointIndices_

typedef ::jsk_pcl_ros::ClusterPointIndices_<std::allocator<void> > ClusterPointIndices;

typedef boost::shared_ptr< ::jsk_pcl_ros::ClusterPointIndices > ClusterPointIndicesPtr;
typedef boost::shared_ptr< ::jsk_pcl_ros::ClusterPointIndices const> ClusterPointIndicesConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_pcl_ros::ClusterPointIndices_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_pcl_ros::ClusterPointIndices_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace jsk_pcl_ros

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'sensor_msgs': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/sensor_msgs/msg'], 'geometry_msgs': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/geometry_msgs/msg'], 'jsk_pcl_ros': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/jsk_pcl_ros/msg'], 'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1329247.2/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg', '/mathworks/devel/sbs/31/pmurali.Brobot.j1329247.2/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'pcl_msgs': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/pcl_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::jsk_pcl_ros::ClusterPointIndices_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_pcl_ros::ClusterPointIndices_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_pcl_ros::ClusterPointIndices_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_pcl_ros::ClusterPointIndices_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_pcl_ros::ClusterPointIndices_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_pcl_ros::ClusterPointIndices_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_pcl_ros::ClusterPointIndices_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d43e94ea5e491effac7685a42b7b9d14";
  }

  static const char* value(const ::jsk_pcl_ros::ClusterPointIndices_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd43e94ea5e491effULL;
  static const uint64_t static_value2 = 0xac7685a42b7b9d14ULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_pcl_ros::ClusterPointIndices_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_pcl_ros/ClusterPointIndices";
  }

  static const char* value(const ::jsk_pcl_ros::ClusterPointIndices_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_pcl_ros::ClusterPointIndices_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"pcl_msgs/PointIndices[] cluster_indices \n"
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
"MSG: pcl_msgs/PointIndices\n"
"Header header\n"
"int32[] indices\n"
"\n"
;
  }

  static const char* value(const ::jsk_pcl_ros::ClusterPointIndices_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_pcl_ros::ClusterPointIndices_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.cluster_indices);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ClusterPointIndices_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_pcl_ros::ClusterPointIndices_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_pcl_ros::ClusterPointIndices_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "cluster_indices[]" << std::endl;
    for (size_t i = 0; i < v.cluster_indices.size(); ++i)
    {
      s << indent << "  cluster_indices[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::pcl_msgs::PointIndices_<ContainerAllocator> >::stream(s, indent + "    ", v.cluster_indices[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_PCL_ROS_MESSAGE_CLUSTERPOINTINDICES_H
