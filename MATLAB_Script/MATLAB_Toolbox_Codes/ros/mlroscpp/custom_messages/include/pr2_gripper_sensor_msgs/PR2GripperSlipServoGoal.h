// Generated by gencpp from file pr2_gripper_sensor_msgs/PR2GripperSlipServoGoal.msg
// DO NOT EDIT!


#ifndef PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPERSLIPSERVOGOAL_H
#define PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPERSLIPSERVOGOAL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <pr2_gripper_sensor_msgs/PR2GripperSlipServoCommand.h>

namespace pr2_gripper_sensor_msgs
{
template <class ContainerAllocator>
struct PR2GripperSlipServoGoal_
{
  typedef PR2GripperSlipServoGoal_<ContainerAllocator> Type;

  PR2GripperSlipServoGoal_()
    : command()  {
    }
  PR2GripperSlipServoGoal_(const ContainerAllocator& _alloc)
    : command(_alloc)  {
  (void)_alloc;
    }



   typedef  ::pr2_gripper_sensor_msgs::PR2GripperSlipServoCommand_<ContainerAllocator>  _command_type;
  _command_type command;





  typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoGoal_<ContainerAllocator> const> ConstPtr;

}; // struct PR2GripperSlipServoGoal_

typedef ::pr2_gripper_sensor_msgs::PR2GripperSlipServoGoal_<std::allocator<void> > PR2GripperSlipServoGoal;

typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoGoal > PR2GripperSlipServoGoalPtr;
typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoGoal const> PR2GripperSlipServoGoalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pr2_gripper_sensor_msgs::PR2GripperSlipServoGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoGoal_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace pr2_gripper_sensor_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'pr2_gripper_sensor_msgs': ['/local-ssd1/All_Custom_Msgs/PendingActionPackagesPart2/matlab_msg_gen_ros1/glnxa64/src/pr2_gripper_sensor_msgs/msg', '/local-ssd1/All_Custom_Msgs/PendingActionPackagesPart2/matlab_msg_gen_ros1/glnxa64/devel/share/pr2_gripper_sensor_msgs/msg'], 'actionlib': ['/mathworks/devel/sbs/31/hakakarl.Brobot.j1347395/matlab/sys/ros1/glnxa64/ros1/share/actionlib/cmake/../msg'], 'std_msgs': ['/mathworks/devel/sbs/31/hakakarl.Brobot.j1347395/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/mathworks/devel/sbs/31/hakakarl.Brobot.j1347395/matlab/sys/ros1/glnxa64/ros1/share/actionlib_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoGoal_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bf76e656d304158c04ab279db7cefc85";
  }

  static const char* value(const ::pr2_gripper_sensor_msgs::PR2GripperSlipServoGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbf76e656d304158cULL;
  static const uint64_t static_value2 = 0x04ab279db7cefc85ULL;
};

template<class ContainerAllocator>
struct DataType< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pr2_gripper_sensor_msgs/PR2GripperSlipServoGoal";
  }

  static const char* value(const ::pr2_gripper_sensor_msgs::PR2GripperSlipServoGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"# Action to launch the gripper into slip servoing mode \n"
"\n"
"#goals\n"
"PR2GripperSlipServoCommand command\n"
"\n"
"================================================================================\n"
"MSG: pr2_gripper_sensor_msgs/PR2GripperSlipServoCommand\n"
"# this command is currently blank, but may see additional variable\n"
"# additions in the future\n"
"\n"
"# see the param server documentation for a list of variables that effect\n"
"# slip servo performance\n"
;
  }

  static const char* value(const ::pr2_gripper_sensor_msgs::PR2GripperSlipServoGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.command);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PR2GripperSlipServoGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pr2_gripper_sensor_msgs::PR2GripperSlipServoGoal_<ContainerAllocator>& v)
  {
    s << indent << "command: ";
    s << std::endl;
    Printer< ::pr2_gripper_sensor_msgs::PR2GripperSlipServoCommand_<ContainerAllocator> >::stream(s, indent + "  ", v.command);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPERSLIPSERVOGOAL_H