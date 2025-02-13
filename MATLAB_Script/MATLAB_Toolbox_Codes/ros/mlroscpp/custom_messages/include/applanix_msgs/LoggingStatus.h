// Generated by gencpp from file applanix_msgs/LoggingStatus.msg
// DO NOT EDIT!


#ifndef APPLANIX_MSGS_MESSAGE_LOGGINGSTATUS_H
#define APPLANIX_MSGS_MESSAGE_LOGGINGSTATUS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <applanix_msgs/TimeDistance.h>

namespace applanix_msgs
{
template <class ContainerAllocator>
struct LoggingStatus_
{
  typedef LoggingStatus_<ContainerAllocator> Type;

  LoggingStatus_()
    : td()
    , disk_kb_remaining(0)
    , disk_kb_logged(0)
    , disk_time_remaining(0)
    , disk_kb_total(0)
    , state(0)  {
    }
  LoggingStatus_(const ContainerAllocator& _alloc)
    : td(_alloc)
    , disk_kb_remaining(0)
    , disk_kb_logged(0)
    , disk_time_remaining(0)
    , disk_kb_total(0)
    , state(0)  {
  (void)_alloc;
    }



   typedef  ::applanix_msgs::TimeDistance_<ContainerAllocator>  _td_type;
  _td_type td;

   typedef uint32_t _disk_kb_remaining_type;
  _disk_kb_remaining_type disk_kb_remaining;

   typedef uint32_t _disk_kb_logged_type;
  _disk_kb_logged_type disk_kb_logged;

   typedef uint32_t _disk_time_remaining_type;
  _disk_time_remaining_type disk_time_remaining;

   typedef uint32_t _disk_kb_total_type;
  _disk_kb_total_type disk_kb_total;

   typedef uint8_t _state_type;
  _state_type state;



  enum {
    STATE_STANDBY = 0u,
    STATE_LOGGING = 1u,
    STATE_BUFFERING = 2u,
    STATE_INVALID = 255u,
  };


  typedef boost::shared_ptr< ::applanix_msgs::LoggingStatus_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::applanix_msgs::LoggingStatus_<ContainerAllocator> const> ConstPtr;

}; // struct LoggingStatus_

typedef ::applanix_msgs::LoggingStatus_<std::allocator<void> > LoggingStatus;

typedef boost::shared_ptr< ::applanix_msgs::LoggingStatus > LoggingStatusPtr;
typedef boost::shared_ptr< ::applanix_msgs::LoggingStatus const> LoggingStatusConstPtr;

// constants requiring out of line definition

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::applanix_msgs::LoggingStatus_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::applanix_msgs::LoggingStatus_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace applanix_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/geometry_msgs/msg'], 'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1329247.2/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'applanix_msgs': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/applanix_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::applanix_msgs::LoggingStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::applanix_msgs::LoggingStatus_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::applanix_msgs::LoggingStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::applanix_msgs::LoggingStatus_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::applanix_msgs::LoggingStatus_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::applanix_msgs::LoggingStatus_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::applanix_msgs::LoggingStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bc37af667689cff6ff70d20f8053fa45";
  }

  static const char* value(const ::applanix_msgs::LoggingStatus_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbc37af667689cff6ULL;
  static const uint64_t static_value2 = 0xff70d20f8053fa45ULL;
};

template<class ContainerAllocator>
struct DataType< ::applanix_msgs::LoggingStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "applanix_msgs/LoggingStatus";
  }

  static const char* value(const ::applanix_msgs::LoggingStatus_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::applanix_msgs::LoggingStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "TimeDistance td\n"
"\n"
"uint32 disk_kb_remaining\n"
"uint32 disk_kb_logged\n"
"uint32 disk_time_remaining\n"
"uint32 disk_kb_total\n"
"\n"
"uint8 STATE_STANDBY=0\n"
"uint8 STATE_LOGGING=1\n"
"uint8 STATE_BUFFERING=2\n"
"uint8 STATE_INVALID=255\n"
"uint8 state\n"
"\n"
"================================================================================\n"
"MSG: applanix_msgs/TimeDistance\n"
"float64 time1\n"
"float64 time2\n"
"float64 distance\n"
"uint8 time_types\n"
"uint8 distance_type\n"
;
  }

  static const char* value(const ::applanix_msgs::LoggingStatus_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::applanix_msgs::LoggingStatus_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.td);
      stream.next(m.disk_kb_remaining);
      stream.next(m.disk_kb_logged);
      stream.next(m.disk_time_remaining);
      stream.next(m.disk_kb_total);
      stream.next(m.state);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LoggingStatus_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::applanix_msgs::LoggingStatus_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::applanix_msgs::LoggingStatus_<ContainerAllocator>& v)
  {
    s << indent << "td: ";
    s << std::endl;
    Printer< ::applanix_msgs::TimeDistance_<ContainerAllocator> >::stream(s, indent + "  ", v.td);
    s << indent << "disk_kb_remaining: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.disk_kb_remaining);
    s << indent << "disk_kb_logged: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.disk_kb_logged);
    s << indent << "disk_time_remaining: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.disk_time_remaining);
    s << indent << "disk_kb_total: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.disk_kb_total);
    s << indent << "state: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.state);
  }
};

} // namespace message_operations
} // namespace ros

#endif // APPLANIX_MSGS_MESSAGE_LOGGINGSTATUS_H
