// Generated by gencpp from file robotnik_msgs/BatteryDockingStatusStamped.msg
// DO NOT EDIT!


#ifndef ROBOTNIK_MSGS_MESSAGE_BATTERYDOCKINGSTATUSSTAMPED_H
#define ROBOTNIK_MSGS_MESSAGE_BATTERYDOCKINGSTATUSSTAMPED_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <robotnik_msgs/BatteryDockingStatus.h>

namespace robotnik_msgs
{
template <class ContainerAllocator>
struct BatteryDockingStatusStamped_
{
  typedef BatteryDockingStatusStamped_<ContainerAllocator> Type;

  BatteryDockingStatusStamped_()
    : header()
    , status()  {
    }
  BatteryDockingStatusStamped_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , status(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::robotnik_msgs::BatteryDockingStatus_<ContainerAllocator>  _status_type;
  _status_type status;





  typedef boost::shared_ptr< ::robotnik_msgs::BatteryDockingStatusStamped_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robotnik_msgs::BatteryDockingStatusStamped_<ContainerAllocator> const> ConstPtr;

}; // struct BatteryDockingStatusStamped_

typedef ::robotnik_msgs::BatteryDockingStatusStamped_<std::allocator<void> > BatteryDockingStatusStamped;

typedef boost::shared_ptr< ::robotnik_msgs::BatteryDockingStatusStamped > BatteryDockingStatusStampedPtr;
typedef boost::shared_ptr< ::robotnik_msgs::BatteryDockingStatusStamped const> BatteryDockingStatusStampedConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robotnik_msgs::BatteryDockingStatusStamped_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robotnik_msgs::BatteryDockingStatusStamped_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::robotnik_msgs::BatteryDockingStatusStamped_<ContainerAllocator1> & lhs, const ::robotnik_msgs::BatteryDockingStatusStamped_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.status == rhs.status;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::robotnik_msgs::BatteryDockingStatusStamped_<ContainerAllocator1> & lhs, const ::robotnik_msgs::BatteryDockingStatusStamped_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace robotnik_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::robotnik_msgs::BatteryDockingStatusStamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robotnik_msgs::BatteryDockingStatusStamped_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robotnik_msgs::BatteryDockingStatusStamped_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robotnik_msgs::BatteryDockingStatusStamped_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotnik_msgs::BatteryDockingStatusStamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotnik_msgs::BatteryDockingStatusStamped_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robotnik_msgs::BatteryDockingStatusStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f9b376e82e9d778484349573af188b1d";
  }

  static const char* value(const ::robotnik_msgs::BatteryDockingStatusStamped_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf9b376e82e9d7784ULL;
  static const uint64_t static_value2 = 0x84349573af188b1dULL;
};

template<class ContainerAllocator>
struct DataType< ::robotnik_msgs::BatteryDockingStatusStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robotnik_msgs/BatteryDockingStatusStamped";
  }

  static const char* value(const ::robotnik_msgs::BatteryDockingStatusStamped_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robotnik_msgs::BatteryDockingStatusStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"BatteryDockingStatus status\n"
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
"MSG: robotnik_msgs/BatteryDockingStatus\n"
"# Modes of operation:\n"
"# no docking station contacts\n"
"string MODE_DISABLED=disabled\n"
"# Unattended relay detection & activation with no inputs/outputs feedback. Done by the hw\n"
"string MODE_AUTO_HW=automatic_hw\n"
"# Unattended relay detection & activation with inputs/outputs feedback. Done by the sw\n"
"string MODE_AUTO_SW=automatic_sw\n"
"# Unattended relay detection & and manual activation of the charging relay\n"
"string MODE_MANUAL_SW=manual_sw\n"
"\n"
"string operation_mode\n"
"	\n"
"bool contact_relay_status	# shows if there's contact with the charger\n"
"bool charger_relay_status   # shows if the relay for the charge is active or not\n"
;
  }

  static const char* value(const ::robotnik_msgs::BatteryDockingStatusStamped_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robotnik_msgs::BatteryDockingStatusStamped_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct BatteryDockingStatusStamped_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robotnik_msgs::BatteryDockingStatusStamped_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robotnik_msgs::BatteryDockingStatusStamped_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "status: ";
    s << std::endl;
    Printer< ::robotnik_msgs::BatteryDockingStatus_<ContainerAllocator> >::stream(s, indent + "  ", v.status);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOTNIK_MSGS_MESSAGE_BATTERYDOCKINGSTATUSSTAMPED_H