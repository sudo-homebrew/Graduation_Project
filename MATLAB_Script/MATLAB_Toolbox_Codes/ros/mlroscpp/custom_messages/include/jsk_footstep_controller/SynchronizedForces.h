// Generated by gencpp from file jsk_footstep_controller/SynchronizedForces.msg
// DO NOT EDIT!


#ifndef JSK_FOOTSTEP_CONTROLLER_MESSAGE_SYNCHRONIZEDFORCES_H
#define JSK_FOOTSTEP_CONTROLLER_MESSAGE_SYNCHRONIZEDFORCES_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PointStamped.h>

namespace jsk_footstep_controller
{
template <class ContainerAllocator>
struct SynchronizedForces_
{
  typedef SynchronizedForces_<ContainerAllocator> Type;

  SynchronizedForces_()
    : header()
    , lleg_force()
    , rleg_force()
    , joint_angles()
    , zmp()  {
    }
  SynchronizedForces_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , lleg_force(_alloc)
    , rleg_force(_alloc)
    , joint_angles(_alloc)
    , zmp(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::geometry_msgs::WrenchStamped_<ContainerAllocator>  _lleg_force_type;
  _lleg_force_type lleg_force;

   typedef  ::geometry_msgs::WrenchStamped_<ContainerAllocator>  _rleg_force_type;
  _rleg_force_type rleg_force;

   typedef  ::sensor_msgs::JointState_<ContainerAllocator>  _joint_angles_type;
  _joint_angles_type joint_angles;

   typedef  ::geometry_msgs::PointStamped_<ContainerAllocator>  _zmp_type;
  _zmp_type zmp;





  typedef boost::shared_ptr< ::jsk_footstep_controller::SynchronizedForces_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_footstep_controller::SynchronizedForces_<ContainerAllocator> const> ConstPtr;

}; // struct SynchronizedForces_

typedef ::jsk_footstep_controller::SynchronizedForces_<std::allocator<void> > SynchronizedForces;

typedef boost::shared_ptr< ::jsk_footstep_controller::SynchronizedForces > SynchronizedForcesPtr;
typedef boost::shared_ptr< ::jsk_footstep_controller::SynchronizedForces const> SynchronizedForcesConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_footstep_controller::SynchronizedForces_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_footstep_controller::SynchronizedForces_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::jsk_footstep_controller::SynchronizedForces_<ContainerAllocator1> & lhs, const ::jsk_footstep_controller::SynchronizedForces_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.lleg_force == rhs.lleg_force &&
    lhs.rleg_force == rhs.rleg_force &&
    lhs.joint_angles == rhs.joint_angles &&
    lhs.zmp == rhs.zmp;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::jsk_footstep_controller::SynchronizedForces_<ContainerAllocator1> & lhs, const ::jsk_footstep_controller::SynchronizedForces_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace jsk_footstep_controller

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::jsk_footstep_controller::SynchronizedForces_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_footstep_controller::SynchronizedForces_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_footstep_controller::SynchronizedForces_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_footstep_controller::SynchronizedForces_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_footstep_controller::SynchronizedForces_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_footstep_controller::SynchronizedForces_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_footstep_controller::SynchronizedForces_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9f34791d0775ccd699ccdfdb8b823128";
  }

  static const char* value(const ::jsk_footstep_controller::SynchronizedForces_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9f34791d0775ccd6ULL;
  static const uint64_t static_value2 = 0x99ccdfdb8b823128ULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_footstep_controller::SynchronizedForces_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_footstep_controller/SynchronizedForces";
  }

  static const char* value(const ::jsk_footstep_controller::SynchronizedForces_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_footstep_controller::SynchronizedForces_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"geometry_msgs/WrenchStamped lleg_force\n"
"geometry_msgs/WrenchStamped rleg_force\n"
"sensor_msgs/JointState joint_angles\n"
"geometry_msgs/PointStamped zmp\n"
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
"MSG: geometry_msgs/WrenchStamped\n"
"# A wrench with reference coordinate frame and timestamp\n"
"Header header\n"
"Wrench wrench\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Wrench\n"
"# This represents force in free space, separated into\n"
"# its linear and angular parts.\n"
"Vector3  force\n"
"Vector3  torque\n"
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
"MSG: sensor_msgs/JointState\n"
"# This is a message that holds data to describe the state of a set of torque controlled joints. \n"
"#\n"
"# The state of each joint (revolute or prismatic) is defined by:\n"
"#  * the position of the joint (rad or m),\n"
"#  * the velocity of the joint (rad/s or m/s) and \n"
"#  * the effort that is applied in the joint (Nm or N).\n"
"#\n"
"# Each joint is uniquely identified by its name\n"
"# The header specifies the time at which the joint states were recorded. All the joint states\n"
"# in one message have to be recorded at the same time.\n"
"#\n"
"# This message consists of a multiple arrays, one for each part of the joint state. \n"
"# The goal is to make each of the fields optional. When e.g. your joints have no\n"
"# effort associated with them, you can leave the effort array empty. \n"
"#\n"
"# All arrays in this message should have the same size, or be empty.\n"
"# This is the only way to uniquely associate the joint name with the correct\n"
"# states.\n"
"\n"
"\n"
"Header header\n"
"\n"
"string[] name\n"
"float64[] position\n"
"float64[] velocity\n"
"float64[] effort\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/PointStamped\n"
"# This represents a Point with reference coordinate frame and timestamp\n"
"Header header\n"
"Point point\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::jsk_footstep_controller::SynchronizedForces_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_footstep_controller::SynchronizedForces_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.lleg_force);
      stream.next(m.rleg_force);
      stream.next(m.joint_angles);
      stream.next(m.zmp);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SynchronizedForces_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_footstep_controller::SynchronizedForces_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_footstep_controller::SynchronizedForces_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "lleg_force: ";
    s << std::endl;
    Printer< ::geometry_msgs::WrenchStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.lleg_force);
    s << indent << "rleg_force: ";
    s << std::endl;
    Printer< ::geometry_msgs::WrenchStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.rleg_force);
    s << indent << "joint_angles: ";
    s << std::endl;
    Printer< ::sensor_msgs::JointState_<ContainerAllocator> >::stream(s, indent + "  ", v.joint_angles);
    s << indent << "zmp: ";
    s << std::endl;
    Printer< ::geometry_msgs::PointStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.zmp);
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_FOOTSTEP_CONTROLLER_MESSAGE_SYNCHRONIZEDFORCES_H