// Generated by gencpp from file rmp_msgs/RMPFeedback.msg
// DO NOT EDIT!


#ifndef RMP_MSGS_MESSAGE_RMPFEEDBACK_H
#define RMP_MSGS_MESSAGE_RMPFEEDBACK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace rmp_msgs
{
template <class ContainerAllocator>
struct RMPFeedback_
{
  typedef RMPFeedback_<ContainerAllocator> Type;

  RMPFeedback_()
    : header()
    , sensor_items()
    , sensor_values()
    , fault_status_items()
    , fault_status_values()
    , ip_info()
    , ip_values()  {
    }
  RMPFeedback_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , sensor_items(_alloc)
    , sensor_values(_alloc)
    , fault_status_items(_alloc)
    , fault_status_values(_alloc)
    , ip_info(_alloc)
    , ip_values(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _sensor_items_type;
  _sensor_items_type sensor_items;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _sensor_values_type;
  _sensor_values_type sensor_values;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _fault_status_items_type;
  _fault_status_items_type fault_status_items;

   typedef std::vector<uint32_t, typename ContainerAllocator::template rebind<uint32_t>::other >  _fault_status_values_type;
  _fault_status_values_type fault_status_values;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _ip_info_type;
  _ip_info_type ip_info;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _ip_values_type;
  _ip_values_type ip_values;





  typedef boost::shared_ptr< ::rmp_msgs::RMPFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rmp_msgs::RMPFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct RMPFeedback_

typedef ::rmp_msgs::RMPFeedback_<std::allocator<void> > RMPFeedback;

typedef boost::shared_ptr< ::rmp_msgs::RMPFeedback > RMPFeedbackPtr;
typedef boost::shared_ptr< ::rmp_msgs::RMPFeedback const> RMPFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rmp_msgs::RMPFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rmp_msgs::RMPFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::rmp_msgs::RMPFeedback_<ContainerAllocator1> & lhs, const ::rmp_msgs::RMPFeedback_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.sensor_items == rhs.sensor_items &&
    lhs.sensor_values == rhs.sensor_values &&
    lhs.fault_status_items == rhs.fault_status_items &&
    lhs.fault_status_values == rhs.fault_status_values &&
    lhs.ip_info == rhs.ip_info &&
    lhs.ip_values == rhs.ip_values;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::rmp_msgs::RMPFeedback_<ContainerAllocator1> & lhs, const ::rmp_msgs::RMPFeedback_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace rmp_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::rmp_msgs::RMPFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rmp_msgs::RMPFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rmp_msgs::RMPFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rmp_msgs::RMPFeedback_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rmp_msgs::RMPFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rmp_msgs::RMPFeedback_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rmp_msgs::RMPFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8688d7b0a2904002e7cc9d6c90a32697";
  }

  static const char* value(const ::rmp_msgs::RMPFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8688d7b0a2904002ULL;
  static const uint64_t static_value2 = 0xe7cc9d6c90a32697ULL;
};

template<class ContainerAllocator>
struct DataType< ::rmp_msgs::RMPFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rmp_msgs/RMPFeedback";
  }

  static const char* value(const ::rmp_msgs::RMPFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rmp_msgs::RMPFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Divides RMP feedback items and values into float and int categories.\n"
"# Items are the names of the feedback parameter and values are the numerical values.\n"
"Header header\n"
"string[] sensor_items\n"
"float32[] sensor_values\n"
"string[] fault_status_items\n"
"uint32[] fault_status_values\n"
"string[] ip_info\n"
"string[] ip_values\n"
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

  static const char* value(const ::rmp_msgs::RMPFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rmp_msgs::RMPFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.sensor_items);
      stream.next(m.sensor_values);
      stream.next(m.fault_status_items);
      stream.next(m.fault_status_values);
      stream.next(m.ip_info);
      stream.next(m.ip_values);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RMPFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rmp_msgs::RMPFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rmp_msgs::RMPFeedback_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "sensor_items[]" << std::endl;
    for (size_t i = 0; i < v.sensor_items.size(); ++i)
    {
      s << indent << "  sensor_items[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.sensor_items[i]);
    }
    s << indent << "sensor_values[]" << std::endl;
    for (size_t i = 0; i < v.sensor_values.size(); ++i)
    {
      s << indent << "  sensor_values[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.sensor_values[i]);
    }
    s << indent << "fault_status_items[]" << std::endl;
    for (size_t i = 0; i < v.fault_status_items.size(); ++i)
    {
      s << indent << "  fault_status_items[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.fault_status_items[i]);
    }
    s << indent << "fault_status_values[]" << std::endl;
    for (size_t i = 0; i < v.fault_status_values.size(); ++i)
    {
      s << indent << "  fault_status_values[" << i << "]: ";
      Printer<uint32_t>::stream(s, indent + "  ", v.fault_status_values[i]);
    }
    s << indent << "ip_info[]" << std::endl;
    for (size_t i = 0; i < v.ip_info.size(); ++i)
    {
      s << indent << "  ip_info[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.ip_info[i]);
    }
    s << indent << "ip_values[]" << std::endl;
    for (size_t i = 0; i < v.ip_values.size(); ++i)
    {
      s << indent << "  ip_values[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.ip_values[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // RMP_MSGS_MESSAGE_RMPFEEDBACK_H