// Generated by gencpp from file pr2_controllers_msgs/PointHeadGoal.msg
// DO NOT EDIT!


#ifndef PR2_CONTROLLERS_MSGS_MESSAGE_POINTHEADGOAL_H
#define PR2_CONTROLLERS_MSGS_MESSAGE_POINTHEADGOAL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>

namespace pr2_controllers_msgs
{
template <class ContainerAllocator>
struct PointHeadGoal_
{
  typedef PointHeadGoal_<ContainerAllocator> Type;

  PointHeadGoal_()
    : target()
    , pointing_axis()
    , pointing_frame()
    , min_duration()
    , max_velocity(0.0)  {
    }
  PointHeadGoal_(const ContainerAllocator& _alloc)
    : target(_alloc)
    , pointing_axis(_alloc)
    , pointing_frame(_alloc)
    , min_duration()
    , max_velocity(0.0)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::PointStamped_<ContainerAllocator>  _target_type;
  _target_type target;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _pointing_axis_type;
  _pointing_axis_type pointing_axis;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _pointing_frame_type;
  _pointing_frame_type pointing_frame;

   typedef ros::Duration _min_duration_type;
  _min_duration_type min_duration;

   typedef double _max_velocity_type;
  _max_velocity_type max_velocity;





  typedef boost::shared_ptr< ::pr2_controllers_msgs::PointHeadGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pr2_controllers_msgs::PointHeadGoal_<ContainerAllocator> const> ConstPtr;

}; // struct PointHeadGoal_

typedef ::pr2_controllers_msgs::PointHeadGoal_<std::allocator<void> > PointHeadGoal;

typedef boost::shared_ptr< ::pr2_controllers_msgs::PointHeadGoal > PointHeadGoalPtr;
typedef boost::shared_ptr< ::pr2_controllers_msgs::PointHeadGoal const> PointHeadGoalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pr2_controllers_msgs::PointHeadGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pr2_controllers_msgs::PointHeadGoal_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::pr2_controllers_msgs::PointHeadGoal_<ContainerAllocator1> & lhs, const ::pr2_controllers_msgs::PointHeadGoal_<ContainerAllocator2> & rhs)
{
  return lhs.target == rhs.target &&
    lhs.pointing_axis == rhs.pointing_axis &&
    lhs.pointing_frame == rhs.pointing_frame &&
    lhs.min_duration == rhs.min_duration &&
    lhs.max_velocity == rhs.max_velocity;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::pr2_controllers_msgs::PointHeadGoal_<ContainerAllocator1> & lhs, const ::pr2_controllers_msgs::PointHeadGoal_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace pr2_controllers_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::pr2_controllers_msgs::PointHeadGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pr2_controllers_msgs::PointHeadGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pr2_controllers_msgs::PointHeadGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pr2_controllers_msgs::PointHeadGoal_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pr2_controllers_msgs::PointHeadGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pr2_controllers_msgs::PointHeadGoal_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pr2_controllers_msgs::PointHeadGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8b92b1cd5e06c8a94c917dc3209a4c1d";
  }

  static const char* value(const ::pr2_controllers_msgs::PointHeadGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8b92b1cd5e06c8a9ULL;
  static const uint64_t static_value2 = 0x4c917dc3209a4c1dULL;
};

template<class ContainerAllocator>
struct DataType< ::pr2_controllers_msgs::PointHeadGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pr2_controllers_msgs/PointHeadGoal";
  }

  static const char* value(const ::pr2_controllers_msgs::PointHeadGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pr2_controllers_msgs::PointHeadGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"geometry_msgs/PointStamped target\n"
"geometry_msgs/Vector3 pointing_axis\n"
"string pointing_frame\n"
"duration min_duration\n"
"float64 max_velocity\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/PointStamped\n"
"# This represents a Point with reference coordinate frame and timestamp\n"
"Header header\n"
"Point point\n"
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
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
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
;
  }

  static const char* value(const ::pr2_controllers_msgs::PointHeadGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pr2_controllers_msgs::PointHeadGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.target);
      stream.next(m.pointing_axis);
      stream.next(m.pointing_frame);
      stream.next(m.min_duration);
      stream.next(m.max_velocity);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PointHeadGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pr2_controllers_msgs::PointHeadGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pr2_controllers_msgs::PointHeadGoal_<ContainerAllocator>& v)
  {
    s << indent << "target: ";
    s << std::endl;
    Printer< ::geometry_msgs::PointStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.target);
    s << indent << "pointing_axis: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.pointing_axis);
    s << indent << "pointing_frame: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.pointing_frame);
    s << indent << "min_duration: ";
    Printer<ros::Duration>::stream(s, indent + "  ", v.min_duration);
    s << indent << "max_velocity: ";
    Printer<double>::stream(s, indent + "  ", v.max_velocity);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PR2_CONTROLLERS_MSGS_MESSAGE_POINTHEADGOAL_H