// Generated by gencpp from file pr2_gripper_sensor_msgs/PR2GripperFindContactAction.msg
// DO NOT EDIT!


#ifndef PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPERFINDCONTACTACTION_H
#define PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPERFINDCONTACTACTION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <pr2_gripper_sensor_msgs/PR2GripperFindContactActionGoal.h>
#include <pr2_gripper_sensor_msgs/PR2GripperFindContactActionResult.h>
#include <pr2_gripper_sensor_msgs/PR2GripperFindContactActionFeedback.h>

namespace pr2_gripper_sensor_msgs
{
template <class ContainerAllocator>
struct PR2GripperFindContactAction_
{
  typedef PR2GripperFindContactAction_<ContainerAllocator> Type;

  PR2GripperFindContactAction_()
    : action_goal()
    , action_result()
    , action_feedback()  {
    }
  PR2GripperFindContactAction_(const ContainerAllocator& _alloc)
    : action_goal(_alloc)
    , action_result(_alloc)
    , action_feedback(_alloc)  {
  (void)_alloc;
    }



   typedef  ::pr2_gripper_sensor_msgs::PR2GripperFindContactActionGoal_<ContainerAllocator>  _action_goal_type;
  _action_goal_type action_goal;

   typedef  ::pr2_gripper_sensor_msgs::PR2GripperFindContactActionResult_<ContainerAllocator>  _action_result_type;
  _action_result_type action_result;

   typedef  ::pr2_gripper_sensor_msgs::PR2GripperFindContactActionFeedback_<ContainerAllocator>  _action_feedback_type;
  _action_feedback_type action_feedback;





  typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperFindContactAction_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperFindContactAction_<ContainerAllocator> const> ConstPtr;

}; // struct PR2GripperFindContactAction_

typedef ::pr2_gripper_sensor_msgs::PR2GripperFindContactAction_<std::allocator<void> > PR2GripperFindContactAction;

typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperFindContactAction > PR2GripperFindContactActionPtr;
typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperFindContactAction const> PR2GripperFindContactActionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pr2_gripper_sensor_msgs::PR2GripperFindContactAction_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pr2_gripper_sensor_msgs::PR2GripperFindContactAction_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace pr2_gripper_sensor_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'pr2_gripper_sensor_msgs': ['/local-ssd1/All_Custom_Msgs/PendingActionPackagesPart2/matlab_msg_gen_ros1/glnxa64/src/pr2_gripper_sensor_msgs/msg', '/local-ssd1/All_Custom_Msgs/PendingActionPackagesPart2/matlab_msg_gen_ros1/glnxa64/devel/share/pr2_gripper_sensor_msgs/msg'], 'actionlib': ['/mathworks/devel/sbs/31/hakakarl.Brobot.j1347395/matlab/sys/ros1/glnxa64/ros1/share/actionlib/cmake/../msg'], 'std_msgs': ['/mathworks/devel/sbs/31/hakakarl.Brobot.j1347395/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/mathworks/devel/sbs/31/hakakarl.Brobot.j1347395/matlab/sys/ros1/glnxa64/ros1/share/actionlib_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::pr2_gripper_sensor_msgs::PR2GripperFindContactAction_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pr2_gripper_sensor_msgs::PR2GripperFindContactAction_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pr2_gripper_sensor_msgs::PR2GripperFindContactAction_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pr2_gripper_sensor_msgs::PR2GripperFindContactAction_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pr2_gripper_sensor_msgs::PR2GripperFindContactAction_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pr2_gripper_sensor_msgs::PR2GripperFindContactAction_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pr2_gripper_sensor_msgs::PR2GripperFindContactAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "99ab2df1bbde46c447b38f28b7896d16";
  }

  static const char* value(const ::pr2_gripper_sensor_msgs::PR2GripperFindContactAction_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x99ab2df1bbde46c4ULL;
  static const uint64_t static_value2 = 0x47b38f28b7896d16ULL;
};

template<class ContainerAllocator>
struct DataType< ::pr2_gripper_sensor_msgs::PR2GripperFindContactAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pr2_gripper_sensor_msgs/PR2GripperFindContactAction";
  }

  static const char* value(const ::pr2_gripper_sensor_msgs::PR2GripperFindContactAction_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pr2_gripper_sensor_msgs::PR2GripperFindContactAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"PR2GripperFindContactActionGoal action_goal\n"
"PR2GripperFindContactActionResult action_result\n"
"PR2GripperFindContactActionFeedback action_feedback\n"
"\n"
"================================================================================\n"
"MSG: pr2_gripper_sensor_msgs/PR2GripperFindContactActionGoal\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
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
"\n"
"================================================================================\n"
"MSG: pr2_gripper_sensor_msgs/PR2GripperFindContactActionResult\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"Header header\n"
"actionlib_msgs/GoalStatus status\n"
"PR2GripperFindContactResult result\n"
"\n"
"================================================================================\n"
"MSG: actionlib_msgs/GoalStatus\n"
"GoalID goal_id\n"
"uint8 status\n"
"uint8 PENDING         = 0   # The goal has yet to be processed by the action server\n"
"uint8 ACTIVE          = 1   # The goal is currently being processed by the action server\n"
"uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing\n"
"                            #   and has since completed its execution (Terminal State)\n"
"uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)\n"
"uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due\n"
"                            #    to some failure (Terminal State)\n"
"uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,\n"
"                            #    because the goal was unattainable or invalid (Terminal State)\n"
"uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing\n"
"                            #    and has not yet completed execution\n"
"uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,\n"
"                            #    but the action server has not yet confirmed that the goal is canceled\n"
"uint8 RECALLED        = 8   # The goal received a cancel request before it started executing\n"
"                            #    and was successfully cancelled (Terminal State)\n"
"uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be\n"
"                            #    sent over the wire by an action server\n"
"\n"
"#Allow for the user to associate a string with GoalStatus for debugging\n"
"string text\n"
"\n"
"\n"
"================================================================================\n"
"MSG: pr2_gripper_sensor_msgs/PR2GripperFindContactResult\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"#results\n"
"PR2GripperFindContactData data\n"
"\n"
"================================================================================\n"
"MSG: pr2_gripper_sensor_msgs/PR2GripperFindContactData\n"
"# Time the data was recorded at\n"
"time stamp\n"
"\n"
"# true when our contact conditions have been met\n"
"# (see PR2GripperFindContact command)\n"
"bool contact_conditions_met\n"
"\n"
"# the finger contact conditions \n"
"# true if the finger experienced a contact event\n"
"#\n"
"# contact events are defined as contact with the fingerpads\n"
"# as either steady-state or high-freq force events\n"
"bool left_fingertip_pad_contact\n"
"bool right_fingertip_pad_contact\n"
"\n"
"# the force experinced by the finger Pads  (N)\n"
"# NOTE:this ignores data from the edges of the finger pressure\n"
"float64 left_fingertip_pad_force\n"
"float64 right_fingertip_pad_force\n"
"\n"
"# the current joint position (m)\n"
"float64 joint_position\n"
"\n"
"# the virtual (parallel) joint effort (N)\n"
"float64 joint_effort\n"
"\n"
"# the control state of our realtime controller\n"
"PR2GripperSensorRTState rtstate\n"
"================================================================================\n"
"MSG: pr2_gripper_sensor_msgs/PR2GripperSensorRTState\n"
"# the control state of our realtime controller\n"
"int8 realtime_controller_state\n"
"\n"
"# predefined values to indicate our realtime_controller_state\n"
"int8 DISABLED = 0\n"
"int8 POSITION_SERVO = 3\n"
"int8 FORCE_SERVO = 4\n"
"int8 FIND_CONTACT = 5\n"
"int8 SLIP_SERVO = 6\n"
"================================================================================\n"
"MSG: pr2_gripper_sensor_msgs/PR2GripperFindContactActionFeedback\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"Header header\n"
"actionlib_msgs/GoalStatus status\n"
"PR2GripperFindContactFeedback feedback\n"
"\n"
"================================================================================\n"
"MSG: pr2_gripper_sensor_msgs/PR2GripperFindContactFeedback\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"# feedback\n"
"PR2GripperFindContactData data\n"
"\n"
"\n"
;
  }

  static const char* value(const ::pr2_gripper_sensor_msgs::PR2GripperFindContactAction_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pr2_gripper_sensor_msgs::PR2GripperFindContactAction_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.action_goal);
      stream.next(m.action_result);
      stream.next(m.action_feedback);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PR2GripperFindContactAction_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pr2_gripper_sensor_msgs::PR2GripperFindContactAction_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pr2_gripper_sensor_msgs::PR2GripperFindContactAction_<ContainerAllocator>& v)
  {
    s << indent << "action_goal: ";
    s << std::endl;
    Printer< ::pr2_gripper_sensor_msgs::PR2GripperFindContactActionGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.action_goal);
    s << indent << "action_result: ";
    s << std::endl;
    Printer< ::pr2_gripper_sensor_msgs::PR2GripperFindContactActionResult_<ContainerAllocator> >::stream(s, indent + "  ", v.action_result);
    s << indent << "action_feedback: ";
    s << std::endl;
    Printer< ::pr2_gripper_sensor_msgs::PR2GripperFindContactActionFeedback_<ContainerAllocator> >::stream(s, indent + "  ", v.action_feedback);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPERFINDCONTACTACTION_H