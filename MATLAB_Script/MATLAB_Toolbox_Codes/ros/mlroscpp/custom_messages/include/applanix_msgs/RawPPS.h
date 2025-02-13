// Generated by gencpp from file applanix_msgs/RawPPS.msg
// DO NOT EDIT!


#ifndef APPLANIX_MSGS_MESSAGE_RAWPPS_H
#define APPLANIX_MSGS_MESSAGE_RAWPPS_H


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
struct RawPPS_
{
  typedef RawPPS_<ContainerAllocator> Type;

  RawPPS_()
    : td()
    , pps_count(0)  {
    }
  RawPPS_(const ContainerAllocator& _alloc)
    : td(_alloc)
    , pps_count(0)  {
  (void)_alloc;
    }



   typedef  ::applanix_msgs::TimeDistance_<ContainerAllocator>  _td_type;
  _td_type td;

   typedef uint32_t _pps_count_type;
  _pps_count_type pps_count;





  typedef boost::shared_ptr< ::applanix_msgs::RawPPS_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::applanix_msgs::RawPPS_<ContainerAllocator> const> ConstPtr;

}; // struct RawPPS_

typedef ::applanix_msgs::RawPPS_<std::allocator<void> > RawPPS;

typedef boost::shared_ptr< ::applanix_msgs::RawPPS > RawPPSPtr;
typedef boost::shared_ptr< ::applanix_msgs::RawPPS const> RawPPSConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::applanix_msgs::RawPPS_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::applanix_msgs::RawPPS_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::applanix_msgs::RawPPS_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::applanix_msgs::RawPPS_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::applanix_msgs::RawPPS_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::applanix_msgs::RawPPS_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::applanix_msgs::RawPPS_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::applanix_msgs::RawPPS_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::applanix_msgs::RawPPS_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e33ff36fd1530481fcda257606892512";
  }

  static const char* value(const ::applanix_msgs::RawPPS_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe33ff36fd1530481ULL;
  static const uint64_t static_value2 = 0xfcda257606892512ULL;
};

template<class ContainerAllocator>
struct DataType< ::applanix_msgs::RawPPS_<ContainerAllocator> >
{
  static const char* value()
  {
    return "applanix_msgs/RawPPS";
  }

  static const char* value(const ::applanix_msgs::RawPPS_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::applanix_msgs::RawPPS_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Group 10003\n"
"TimeDistance td\n"
"uint32 pps_count\n"
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

  static const char* value(const ::applanix_msgs::RawPPS_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::applanix_msgs::RawPPS_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.td);
      stream.next(m.pps_count);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RawPPS_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::applanix_msgs::RawPPS_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::applanix_msgs::RawPPS_<ContainerAllocator>& v)
  {
    s << indent << "td: ";
    s << std::endl;
    Printer< ::applanix_msgs::TimeDistance_<ContainerAllocator> >::stream(s, indent + "  ", v.td);
    s << indent << "pps_count: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.pps_count);
  }
};

} // namespace message_operations
} // namespace ros

#endif // APPLANIX_MSGS_MESSAGE_RAWPPS_H
