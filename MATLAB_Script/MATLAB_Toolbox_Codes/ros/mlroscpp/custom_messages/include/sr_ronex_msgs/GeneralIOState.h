// Generated by gencpp from file sr_ronex_msgs/GeneralIOState.msg
// DO NOT EDIT!


#ifndef SR_RONEX_MSGS_MESSAGE_GENERALIOSTATE_H
#define SR_RONEX_MSGS_MESSAGE_GENERALIOSTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace sr_ronex_msgs
{
template <class ContainerAllocator>
struct GeneralIOState_
{
  typedef GeneralIOState_<ContainerAllocator> Type;

  GeneralIOState_()
    : header()
    , digital()
    , analogue()
    , pwm_clock_divider(0)
    , input_mode()  {
    }
  GeneralIOState_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , digital(_alloc)
    , analogue(_alloc)
    , pwm_clock_divider(0)
    , input_mode(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  _digital_type;
  _digital_type digital;

   typedef std::vector<uint16_t, typename ContainerAllocator::template rebind<uint16_t>::other >  _analogue_type;
  _analogue_type analogue;

   typedef uint16_t _pwm_clock_divider_type;
  _pwm_clock_divider_type pwm_clock_divider;

   typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  _input_mode_type;
  _input_mode_type input_mode;





  typedef boost::shared_ptr< ::sr_ronex_msgs::GeneralIOState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::sr_ronex_msgs::GeneralIOState_<ContainerAllocator> const> ConstPtr;

}; // struct GeneralIOState_

typedef ::sr_ronex_msgs::GeneralIOState_<std::allocator<void> > GeneralIOState;

typedef boost::shared_ptr< ::sr_ronex_msgs::GeneralIOState > GeneralIOStatePtr;
typedef boost::shared_ptr< ::sr_ronex_msgs::GeneralIOState const> GeneralIOStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::sr_ronex_msgs::GeneralIOState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::sr_ronex_msgs::GeneralIOState_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace sr_ronex_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'sr_ronex_msgs': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/sr_ronex_msgs/msg'], 'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1329247.2/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg', '/mathworks/devel/sbs/31/pmurali.Brobot.j1329247.2/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::sr_ronex_msgs::GeneralIOState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sr_ronex_msgs::GeneralIOState_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sr_ronex_msgs::GeneralIOState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sr_ronex_msgs::GeneralIOState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sr_ronex_msgs::GeneralIOState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sr_ronex_msgs::GeneralIOState_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::sr_ronex_msgs::GeneralIOState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0c3ee57b67f445415be2df465ebee265";
  }

  static const char* value(const ::sr_ronex_msgs::GeneralIOState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0c3ee57b67f44541ULL;
  static const uint64_t static_value2 = 0x5be2df465ebee265ULL;
};

template<class ContainerAllocator>
struct DataType< ::sr_ronex_msgs::GeneralIOState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sr_ronex_msgs/GeneralIOState";
  }

  static const char* value(const ::sr_ronex_msgs::GeneralIOState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::sr_ronex_msgs::GeneralIOState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"\n"
"#digital and analogue contain either 6 or 12 elements depending on whether\n"
"# a stacker board is attached to the RoNeX or not.\n"
"bool[] digital\n"
"uint16[] analogue\n"
"uint16 pwm_clock_divider\n"
"bool[] input_mode\n"
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
;
  }

  static const char* value(const ::sr_ronex_msgs::GeneralIOState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::sr_ronex_msgs::GeneralIOState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.digital);
      stream.next(m.analogue);
      stream.next(m.pwm_clock_divider);
      stream.next(m.input_mode);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GeneralIOState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::sr_ronex_msgs::GeneralIOState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::sr_ronex_msgs::GeneralIOState_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "digital[]" << std::endl;
    for (size_t i = 0; i < v.digital.size(); ++i)
    {
      s << indent << "  digital[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.digital[i]);
    }
    s << indent << "analogue[]" << std::endl;
    for (size_t i = 0; i < v.analogue.size(); ++i)
    {
      s << indent << "  analogue[" << i << "]: ";
      Printer<uint16_t>::stream(s, indent + "  ", v.analogue[i]);
    }
    s << indent << "pwm_clock_divider: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.pwm_clock_divider);
    s << indent << "input_mode[]" << std::endl;
    for (size_t i = 0; i < v.input_mode.size(); ++i)
    {
      s << indent << "  input_mode[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.input_mode[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // SR_RONEX_MSGS_MESSAGE_GENERALIOSTATE_H