// Generated by gencpp from file pr2_gripper_sensor_msgs/PR2GripperFindContactActionGoal.msg
// DO NOT EDIT!


#ifndef PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPERFINDCONTACTACTIONGOAL_H
#define PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPERFINDCONTACTACTIONGOAL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <actionlib_msgs/GoalID.h>
#include <pr2_gripper_sensor_msgs/PR2GripperFindContactGoal.h>

namespace pr2_gripper_sensor_msgs
{
template <class ContainerAllocator>
struct PR2GripperFindContactActionGoal_
{
  typedef PR2GripperFindContactActionGoal_<ContainerAllocator> Type;

  PR2GripperFindContactActionGoal_()
    : header()
    , goal_id()
    , goal()  {
    }
  PR2GripperFindContactActionGoal_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , goal_id(_alloc)
    , goal(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::actionlib_msgs::GoalID_<ContainerAllocator>  _goal_id_type;
  _goal_id_type goal_id;

   typedef  ::pr2_gripper_sensor_msgs::PR2GripperFindContactGoal_<ContainerAllocator>  _goal_type;
  _goal_type goal;





  typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperFindContactActionGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperFindContactActionGoal_<ContainerAllocator> const> ConstPtr;

}; // struct PR2GripperFindContactActionGoal_

typedef ::pr2_gripper_sensor_msgs::PR2GripperFindContactActionGoal_<std::allocator<void> > PR2GripperFindContactActionGoal;

typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperFindContactActionGoal > PR2GripperFindContactActionGoalPtr;
typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperFindContactActionGoal const> PR2GripperFindContactActionGoalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pr2_gripper_sensor_msgs::PR2GripperFindContactActionGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pr2_gripper_sensor_msgs::PR2GripperFindContactActionGoal_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace pr2_gripper_sensor_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'pr2_gripper_sensor_msgs': ['/local-ssd1/All_Custom_Msgs/PendingActionPackagesPart2/matlab_msg_gen_ros1/glnxa64/src/pr2_gripper_sensor_msgs/msg', '/local-ssd1/All_Custom_Msgs/PendingActionPackagesPart2/matlab_msg_gen_ros1/glnxa64/devel/share/pr2_gripper_sensor_msgs/msg'], 'actionlib': ['/mathworks/devel/sbs/31/hakakarl.Brobot.j1347395/matlab/sys/ros1/glnxa64/ros1/share/actionlib/cmake/../msg'], 'std_msgs': ['/mathworks/devel/sbs/31/hakakarl.Brobot.j1347395/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/mathworks/devel/sbs/31/hakakarl.Brobot.j1347395/matlab/sys/ros1/glnxa64/ros1/share/actionlib_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::pr2_gripper_sensor_msgs::PR2GripperFindContactActionGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pr2_gripper_sensor_msgs::PR2GripperFindContactActionGoal_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pr2_gripper_sensor_msgs::PR2GripperFindContactActionGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pr2_gripper_sensor_msgs::PR2GripperFindContactActionGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pr2_gripper_sensor_msgs::PR2GripperFindContactActionGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pr2_gripper_sensor_msgs::PR2GripperFindContactActionGoal_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pr2_gripper_sensor_msgs::PR2GripperFindContactActionGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "50fc3f7e604d4e257a2e38e3aa3f204e";
  }

  static const char* value(const ::pr2_gripper_sensor_msgs::PR2GripperFindContactActionGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x50fc3f7e604d4e25ULL;
  static const uint64_t static_value2 = 0x7a2e38e3aa3f204eULL;
};

template<class ContainerAllocator>
struct DataType< ::pr2_gripper_sensor_msgs::PR2GripperFindContactActionGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pr2_gripper_sensor_msgs/PR2GripperFindContactActionGoal";
  }

  static const char* value(const ::pr2_gripper_sensor_msgs::PR2GripperFindContactActionGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pr2_gripper_sensor_msgs::PR2GripperFindContactActionGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"Header header\n"
"actionlib_msgs/GoalID goal_id\n"
"PR2GripperFindContactGoal goal\n"
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
"MSG: actionlib_msgs/GoalID\n"
"# The stamp should store the time at which this goal was requested.\n"
"# It is used by an action server when it tries to preempt all\n"
"# goals that were requested before a certain time\n"
"time stamp\n"
"\n"
"# The id provides a way to associate feedback and\n"
"# result message with specific goal requests. The id\n"
"# specified must be unique.\n"
"string id\n"
"\n"
"\n"
"================================================================================\n"
"MSG: pr2_gripper_sensor_msgs/PR2GripperFindContactGoal\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"# Contact action used to close fingers and find object contacts \n"
"# quickly while still stopping fast in real-time to not damage \n"
"# objects\n"
"\n"
"#goal\n"
"PR2GripperFindContactCommand command\n"
"\n"
"================================================================================\n"
"MSG: pr2_gripper_sensor_msgs/PR2GripperFindContactCommand\n"
"# set true if you want to calibrate the fingertip sensors on the start\n"
"# of the find_contact action. While this is not necessary (and\n"
"# the default value will not calibrate the sensors) for best \n"
"# performance it is recommended that you set this to true each time \n"
"# you are calling find_contact and are confident the fingertips are \n"
"# not touching anything\n"
"# NOTE: SHOULD ONLY BE TRUE WHEN BOTH FINGERS ARE TOUCHING NOTHING\n"
"bool zero_fingertip_sensors\n"
"\n"
"# the finger contact conditions that determine what our goal is\n"
"# Leaving this field blank will result in the robot closing until\n"
"# contact on BOTH fingers is achieved\n"
"int8 contact_conditions\n"
"\n"
"# predefined values for the above contact_conditions variable\n"
"int8 BOTH = 0   # both fingers must make contact\n"
"int8 LEFT = 1   # just the left finger \n"
"int8 RIGHT = 2  # just the right finger\n"
"int8 EITHER = 3 # either finger, we don't care which\n"
;
  }

  static const char* value(const ::pr2_gripper_sensor_msgs::PR2GripperFindContactActionGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pr2_gripper_sensor_msgs::PR2GripperFindContactActionGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.goal_id);
      stream.next(m.goal);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PR2GripperFindContactActionGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pr2_gripper_sensor_msgs::PR2GripperFindContactActionGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pr2_gripper_sensor_msgs::PR2GripperFindContactActionGoal_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "goal_id: ";
    s << std::endl;
    Printer< ::actionlib_msgs::GoalID_<ContainerAllocator> >::stream(s, indent + "  ", v.goal_id);
    s << indent << "goal: ";
    s << std::endl;
    Printer< ::pr2_gripper_sensor_msgs::PR2GripperFindContactGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.goal);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPERFINDCONTACTACTIONGOAL_H