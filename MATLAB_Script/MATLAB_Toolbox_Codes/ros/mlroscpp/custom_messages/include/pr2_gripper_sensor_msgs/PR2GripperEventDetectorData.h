// Generated by gencpp from file pr2_gripper_sensor_msgs/PR2GripperEventDetectorData.msg
// DO NOT EDIT!


#ifndef PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPEREVENTDETECTORDATA_H
#define PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPEREVENTDETECTORDATA_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace pr2_gripper_sensor_msgs
{
template <class ContainerAllocator>
struct PR2GripperEventDetectorData_
{
  typedef PR2GripperEventDetectorData_<ContainerAllocator> Type;

  PR2GripperEventDetectorData_()
    : stamp()
    , trigger_conditions_met(false)
    , slip_event(false)
    , acceleration_event(false)
    , acceleration_vector()  {
      acceleration_vector.assign(0.0);
  }
  PR2GripperEventDetectorData_(const ContainerAllocator& _alloc)
    : stamp()
    , trigger_conditions_met(false)
    , slip_event(false)
    , acceleration_event(false)
    , acceleration_vector()  {
  (void)_alloc;
      acceleration_vector.assign(0.0);
  }



   typedef ros::Time _stamp_type;
  _stamp_type stamp;

   typedef uint8_t _trigger_conditions_met_type;
  _trigger_conditions_met_type trigger_conditions_met;

   typedef uint8_t _slip_event_type;
  _slip_event_type slip_event;

   typedef uint8_t _acceleration_event_type;
  _acceleration_event_type acceleration_event;

   typedef boost::array<double, 3>  _acceleration_vector_type;
  _acceleration_vector_type acceleration_vector;





  typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorData_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorData_<ContainerAllocator> const> ConstPtr;

}; // struct PR2GripperEventDetectorData_

typedef ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorData_<std::allocator<void> > PR2GripperEventDetectorData;

typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorData > PR2GripperEventDetectorDataPtr;
typedef boost::shared_ptr< ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorData const> PR2GripperEventDetectorDataConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorData_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorData_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorData_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorData_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorData_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorData_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorData_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorData_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9536d682ef6215440ecc47846d4117c2";
  }

  static const char* value(const ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorData_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9536d682ef621544ULL;
  static const uint64_t static_value2 = 0x0ecc47846d4117c2ULL;
};

template<class ContainerAllocator>
struct DataType< ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pr2_gripper_sensor_msgs/PR2GripperEventDetectorData";
  }

  static const char* value(const ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorData_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Time the data was recorded at\n"
"time stamp\n"
"\n"
"# true if the trigger conditions have been met \n"
"# (see PR2GripperEventDetectorCommand)\n"
"bool trigger_conditions_met\n"
"\n"
"# true if the pressure sensors detected a slip event\n"
"# slip events occur when the finger pressure sensors\n"
"# high-freq. content exceeds the slip_trigger_magnitude variable\n"
"# (see PR2GripperEventDetectorCommand)\n"
"bool slip_event\n"
"\n"
"# true if the hand-mounted accelerometer detected a contact acceleration\n"
"# acceleration events occur when the palm accelerometer\n"
"# high-freq. content exceeds the acc_trigger_magnitude variable\n"
"# (see PR2GripperEventDetectorCommand)\n"
"bool acceleration_event\n"
"\n"
"# the high-freq acceleration vector that was last seen (x,y,z)\n"
"float64[3] acceleration_vector\n"
;
  }

  static const char* value(const ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorData_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorData_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.stamp);
      stream.next(m.trigger_conditions_met);
      stream.next(m.slip_event);
      stream.next(m.acceleration_event);
      stream.next(m.acceleration_vector);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PR2GripperEventDetectorData_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorData_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pr2_gripper_sensor_msgs::PR2GripperEventDetectorData_<ContainerAllocator>& v)
  {
    s << indent << "stamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.stamp);
    s << indent << "trigger_conditions_met: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.trigger_conditions_met);
    s << indent << "slip_event: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.slip_event);
    s << indent << "acceleration_event: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.acceleration_event);
    s << indent << "acceleration_vector[]" << std::endl;
    for (size_t i = 0; i < v.acceleration_vector.size(); ++i)
    {
      s << indent << "  acceleration_vector[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.acceleration_vector[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // PR2_GRIPPER_SENSOR_MSGS_MESSAGE_PR2GRIPPEREVENTDETECTORDATA_H