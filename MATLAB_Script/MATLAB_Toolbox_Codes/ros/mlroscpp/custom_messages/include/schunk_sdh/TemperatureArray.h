// Generated by gencpp from file schunk_sdh/TemperatureArray.msg
// DO NOT EDIT!


#ifndef SCHUNK_SDH_MESSAGE_TEMPERATUREARRAY_H
#define SCHUNK_SDH_MESSAGE_TEMPERATUREARRAY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace schunk_sdh
{
template <class ContainerAllocator>
struct TemperatureArray_
{
  typedef TemperatureArray_<ContainerAllocator> Type;

  TemperatureArray_()
    : header()
    , name()
    , temperature()  {
    }
  TemperatureArray_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , name(_alloc)
    , temperature(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _name_type;
  _name_type name;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _temperature_type;
  _temperature_type temperature;





  typedef boost::shared_ptr< ::schunk_sdh::TemperatureArray_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::schunk_sdh::TemperatureArray_<ContainerAllocator> const> ConstPtr;

}; // struct TemperatureArray_

typedef ::schunk_sdh::TemperatureArray_<std::allocator<void> > TemperatureArray;

typedef boost::shared_ptr< ::schunk_sdh::TemperatureArray > TemperatureArrayPtr;
typedef boost::shared_ptr< ::schunk_sdh::TemperatureArray const> TemperatureArrayConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::schunk_sdh::TemperatureArray_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::schunk_sdh::TemperatureArray_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::schunk_sdh::TemperatureArray_<ContainerAllocator1> & lhs, const ::schunk_sdh::TemperatureArray_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.name == rhs.name &&
    lhs.temperature == rhs.temperature;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::schunk_sdh::TemperatureArray_<ContainerAllocator1> & lhs, const ::schunk_sdh::TemperatureArray_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace schunk_sdh

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::schunk_sdh::TemperatureArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::schunk_sdh::TemperatureArray_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::schunk_sdh::TemperatureArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::schunk_sdh::TemperatureArray_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::schunk_sdh::TemperatureArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::schunk_sdh::TemperatureArray_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::schunk_sdh::TemperatureArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0aa09ef71eada777ee697d205df8b8f6";
  }

  static const char* value(const ::schunk_sdh::TemperatureArray_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0aa09ef71eada777ULL;
  static const uint64_t static_value2 = 0xee697d205df8b8f6ULL;
};

template<class ContainerAllocator>
struct DataType< ::schunk_sdh::TemperatureArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "schunk_sdh/TemperatureArray";
  }

  static const char* value(const ::schunk_sdh::TemperatureArray_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::schunk_sdh::TemperatureArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"\n"
"string[] name           # list of sensor name\n"
"float64[] temperature   # list of temperature in degree Celcius (°C)\n"
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

  static const char* value(const ::schunk_sdh::TemperatureArray_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::schunk_sdh::TemperatureArray_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.name);
      stream.next(m.temperature);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TemperatureArray_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::schunk_sdh::TemperatureArray_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::schunk_sdh::TemperatureArray_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "name[]" << std::endl;
    for (size_t i = 0; i < v.name.size(); ++i)
    {
      s << indent << "  name[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name[i]);
    }
    s << indent << "temperature[]" << std::endl;
    for (size_t i = 0; i < v.temperature.size(); ++i)
    {
      s << indent << "  temperature[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.temperature[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // SCHUNK_SDH_MESSAGE_TEMPERATUREARRAY_H