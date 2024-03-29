// Generated by gencpp from file hector_worldmodel_msgs/Object.msg
// DO NOT EDIT!


#ifndef HECTOR_WORLDMODEL_MSGS_MESSAGE_OBJECT_H
#define HECTOR_WORLDMODEL_MSGS_MESSAGE_OBJECT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <hector_worldmodel_msgs/ObjectInfo.h>
#include <hector_worldmodel_msgs/ObjectState.h>

namespace hector_worldmodel_msgs
{
template <class ContainerAllocator>
struct Object_
{
  typedef Object_<ContainerAllocator> Type;

  Object_()
    : header()
    , pose()
    , info()
    , state()  {
    }
  Object_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , pose(_alloc)
    , info(_alloc)
    , state(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::geometry_msgs::PoseWithCovariance_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef  ::hector_worldmodel_msgs::ObjectInfo_<ContainerAllocator>  _info_type;
  _info_type info;

   typedef  ::hector_worldmodel_msgs::ObjectState_<ContainerAllocator>  _state_type;
  _state_type state;





  typedef boost::shared_ptr< ::hector_worldmodel_msgs::Object_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hector_worldmodel_msgs::Object_<ContainerAllocator> const> ConstPtr;

}; // struct Object_

typedef ::hector_worldmodel_msgs::Object_<std::allocator<void> > Object;

typedef boost::shared_ptr< ::hector_worldmodel_msgs::Object > ObjectPtr;
typedef boost::shared_ptr< ::hector_worldmodel_msgs::Object const> ObjectConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hector_worldmodel_msgs::Object_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hector_worldmodel_msgs::Object_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace hector_worldmodel_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'sensor_msgs': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/sensor_msgs/msg'], 'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1329247.2/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg', '/mathworks/devel/sbs/31/pmurali.Brobot.j1329247.2/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'hector_worldmodel_msgs': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/hector_worldmodel_msgs/msg'], 'geometry_msgs': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/geometry_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::hector_worldmodel_msgs::Object_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hector_worldmodel_msgs::Object_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hector_worldmodel_msgs::Object_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hector_worldmodel_msgs::Object_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hector_worldmodel_msgs::Object_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hector_worldmodel_msgs::Object_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hector_worldmodel_msgs::Object_<ContainerAllocator> >
{
  static const char* value()
  {
    return "02dea96e80640703553490052f13918d";
  }

  static const char* value(const ::hector_worldmodel_msgs::Object_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x02dea96e80640703ULL;
  static const uint64_t static_value2 = 0x553490052f13918dULL;
};

template<class ContainerAllocator>
struct DataType< ::hector_worldmodel_msgs::Object_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hector_worldmodel_msgs/Object";
  }

  static const char* value(const ::hector_worldmodel_msgs::Object_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hector_worldmodel_msgs::Object_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# hector_worldmodel_msgs/Object\n"
"# This message represents an estimate of an object's pose and identity.\n"
"\n"
"# The header.\n"
"#   stamp: Timestamp of last update.\n"
"#   frame_id: Coordinate frame, in which the pose is given\n"
"Header header\n"
"\n"
"# The pose\n"
"geometry_msgs/PoseWithCovariance pose\n"
"\n"
"# Further information about the object\n"
"ObjectInfo info\n"
"\n"
"# The tracked state of the object\n"
"ObjectState state\n"
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
"MSG: geometry_msgs/PoseWithCovariance\n"
"# This represents a pose in free space with uncertainty.\n"
"\n"
"Pose pose\n"
"\n"
"# Row-major representation of the 6x6 covariance matrix\n"
"# The orientation parameters use a fixed-axis representation.\n"
"# In order, the parameters are:\n"
"# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n"
"float64[36] covariance\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of postion and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
"\n"
"================================================================================\n"
"MSG: hector_worldmodel_msgs/ObjectInfo\n"
"# hector_worldmodel_msgs/ObjectInfo\n"
"# This message contains information about the estimated class affiliation, object id and corresponding support\n"
"\n"
"# A string identifying the object's class (all objects of a class look the same)\n"
"string class_id\n"
"\n"
"# A string identifying the specific object\n"
"string object_id\n"
"\n"
"# A string that contains the name or a description of the specific object\n"
"string name\n"
"\n"
"# The support (degree of belief) of the object's presence given as log odd ratio\n"
"float32 support\n"
"\n"
"\n"
"================================================================================\n"
"MSG: hector_worldmodel_msgs/ObjectState\n"
"# The state of an object estimate used to track\n"
"# states smaller than 0 disable all updates\n"
"\n"
"# Predefined states. Use states smaller than 0 or bigger than 63 for user defined states.\n"
"int8 UNKNOWN = 0\n"
"int8 PENDING = 1\n"
"int8 ACTIVE  = 2\n"
"int8 INACTIVE = 3\n"
"int8 CONFIRMED = -1\n"
"int8 DISCARDED = -2\n"
"int8 APPROACHING = -3\n"
"\n"
"int8 state\n"
;
  }

  static const char* value(const ::hector_worldmodel_msgs::Object_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hector_worldmodel_msgs::Object_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.pose);
      stream.next(m.info);
      stream.next(m.state);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Object_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hector_worldmodel_msgs::Object_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hector_worldmodel_msgs::Object_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::PoseWithCovariance_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "info: ";
    s << std::endl;
    Printer< ::hector_worldmodel_msgs::ObjectInfo_<ContainerAllocator> >::stream(s, indent + "  ", v.info);
    s << indent << "state: ";
    s << std::endl;
    Printer< ::hector_worldmodel_msgs::ObjectState_<ContainerAllocator> >::stream(s, indent + "  ", v.state);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HECTOR_WORLDMODEL_MSGS_MESSAGE_OBJECT_H
