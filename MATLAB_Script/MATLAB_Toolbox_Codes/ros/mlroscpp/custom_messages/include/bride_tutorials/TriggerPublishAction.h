// Generated by gencpp from file bride_tutorials/TriggerPublishAction.msg
// DO NOT EDIT!


#ifndef BRIDE_TUTORIALS_MESSAGE_TRIGGERPUBLISHACTION_H
#define BRIDE_TUTORIALS_MESSAGE_TRIGGERPUBLISHACTION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <bride_tutorials/TriggerPublishActionGoal.h>
#include <bride_tutorials/TriggerPublishActionResult.h>
#include <bride_tutorials/TriggerPublishActionFeedback.h>

namespace bride_tutorials
{
template <class ContainerAllocator>
struct TriggerPublishAction_
{
  typedef TriggerPublishAction_<ContainerAllocator> Type;

  TriggerPublishAction_()
    : action_goal()
    , action_result()
    , action_feedback()  {
    }
  TriggerPublishAction_(const ContainerAllocator& _alloc)
    : action_goal(_alloc)
    , action_result(_alloc)
    , action_feedback(_alloc)  {
  (void)_alloc;
    }



   typedef  ::bride_tutorials::TriggerPublishActionGoal_<ContainerAllocator>  _action_goal_type;
  _action_goal_type action_goal;

   typedef  ::bride_tutorials::TriggerPublishActionResult_<ContainerAllocator>  _action_result_type;
  _action_result_type action_result;

   typedef  ::bride_tutorials::TriggerPublishActionFeedback_<ContainerAllocator>  _action_feedback_type;
  _action_feedback_type action_feedback;





  typedef boost::shared_ptr< ::bride_tutorials::TriggerPublishAction_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::bride_tutorials::TriggerPublishAction_<ContainerAllocator> const> ConstPtr;

}; // struct TriggerPublishAction_

typedef ::bride_tutorials::TriggerPublishAction_<std::allocator<void> > TriggerPublishAction;

typedef boost::shared_ptr< ::bride_tutorials::TriggerPublishAction > TriggerPublishActionPtr;
typedef boost::shared_ptr< ::bride_tutorials::TriggerPublishAction const> TriggerPublishActionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::bride_tutorials::TriggerPublishAction_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::bride_tutorials::TriggerPublishAction_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace bride_tutorials

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'actionlib': ['/mathworks/devel/sbs/31/hakakarl.Brobot.j1347395/matlab/sys/ros1/glnxa64/ros1/share/actionlib/cmake/../msg'], 'std_msgs': ['/mathworks/devel/sbs/31/hakakarl.Brobot.j1347395/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg', '/mathworks/devel/sbs/31/hakakarl.Brobot.j1347395/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/mathworks/devel/sbs/31/hakakarl.Brobot.j1347395/matlab/sys/ros1/glnxa64/ros1/share/actionlib_msgs/cmake/../msg'], 'bride_tutorials': ['/local-ssd1/All_Custom_Msgs/PendingPackagesInitial/matlab_msg_gen_ros1/glnxa64/src/bride_tutorials/msg', '/local-ssd1/All_Custom_Msgs/PendingPackagesInitial/matlab_msg_gen_ros1/glnxa64/devel/share/bride_tutorials/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::bride_tutorials::TriggerPublishAction_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bride_tutorials::TriggerPublishAction_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bride_tutorials::TriggerPublishAction_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bride_tutorials::TriggerPublishAction_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bride_tutorials::TriggerPublishAction_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bride_tutorials::TriggerPublishAction_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::bride_tutorials::TriggerPublishAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "125b6c5b3d3654bddadf2855b275822e";
  }

  static const char* value(const ::bride_tutorials::TriggerPublishAction_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x125b6c5b3d3654bdULL;
  static const uint64_t static_value2 = 0xdadf2855b275822eULL;
};

template<class ContainerAllocator>
struct DataType< ::bride_tutorials::TriggerPublishAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bride_tutorials/TriggerPublishAction";
  }

  static const char* value(const ::bride_tutorials::TriggerPublishAction_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::bride_tutorials::TriggerPublishAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"TriggerPublishActionGoal action_goal\n"
"TriggerPublishActionResult action_result\n"
"TriggerPublishActionFeedback action_feedback\n"
"\n"
"================================================================================\n"
"MSG: bride_tutorials/TriggerPublishActionGoal\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"Header header\n"
"actionlib_msgs/GoalID goal_id\n"
"TriggerPublishGoal goal\n"
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
"MSG: bride_tutorials/TriggerPublishGoal\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"# Define the goal\n"
"uint32 test\n"
"\n"
"================================================================================\n"
"MSG: bride_tutorials/TriggerPublishActionResult\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"Header header\n"
"actionlib_msgs/GoalStatus status\n"
"TriggerPublishResult result\n"
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
"MSG: bride_tutorials/TriggerPublishResult\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"# Define the result\n"
"\n"
"================================================================================\n"
"MSG: bride_tutorials/TriggerPublishActionFeedback\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"Header header\n"
"actionlib_msgs/GoalStatus status\n"
"TriggerPublishFeedback feedback\n"
"\n"
"================================================================================\n"
"MSG: bride_tutorials/TriggerPublishFeedback\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"# Define a feedback message\n"
"\n"
;
  }

  static const char* value(const ::bride_tutorials::TriggerPublishAction_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::bride_tutorials::TriggerPublishAction_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.action_goal);
      stream.next(m.action_result);
      stream.next(m.action_feedback);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TriggerPublishAction_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::bride_tutorials::TriggerPublishAction_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::bride_tutorials::TriggerPublishAction_<ContainerAllocator>& v)
  {
    s << indent << "action_goal: ";
    s << std::endl;
    Printer< ::bride_tutorials::TriggerPublishActionGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.action_goal);
    s << indent << "action_result: ";
    s << std::endl;
    Printer< ::bride_tutorials::TriggerPublishActionResult_<ContainerAllocator> >::stream(s, indent + "  ", v.action_result);
    s << indent << "action_feedback: ";
    s << std::endl;
    Printer< ::bride_tutorials::TriggerPublishActionFeedback_<ContainerAllocator> >::stream(s, indent + "  ", v.action_feedback);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BRIDE_TUTORIALS_MESSAGE_TRIGGERPUBLISHACTION_H
