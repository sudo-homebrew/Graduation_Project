// Generated by gencpp from file clearpath_base/PowerStatus.msg
// DO NOT EDIT!


#ifndef CLEARPATH_BASE_MESSAGE_POWERSTATUS_H
#define CLEARPATH_BASE_MESSAGE_POWERSTATUS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <clearpath_base/PowerSource.h>

namespace clearpath_base
{
template <class ContainerAllocator>
struct PowerStatus_
{
  typedef PowerStatus_<ContainerAllocator> Type;

  PowerStatus_()
    : header()
    , sources()  {
    }
  PowerStatus_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , sources(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::clearpath_base::PowerSource_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::clearpath_base::PowerSource_<ContainerAllocator> >::other >  _sources_type;
  _sources_type sources;





  typedef boost::shared_ptr< ::clearpath_base::PowerStatus_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::clearpath_base::PowerStatus_<ContainerAllocator> const> ConstPtr;

}; // struct PowerStatus_

typedef ::clearpath_base::PowerStatus_<std::allocator<void> > PowerStatus;

typedef boost::shared_ptr< ::clearpath_base::PowerStatus > PowerStatusPtr;
typedef boost::shared_ptr< ::clearpath_base::PowerStatus const> PowerStatusConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::clearpath_base::PowerStatus_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::clearpath_base::PowerStatus_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::clearpath_base::PowerStatus_<ContainerAllocator1> & lhs, const ::clearpath_base::PowerStatus_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.sources == rhs.sources;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::clearpath_base::PowerStatus_<ContainerAllocator1> & lhs, const ::clearpath_base::PowerStatus_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace clearpath_base

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::clearpath_base::PowerStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::clearpath_base::PowerStatus_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::clearpath_base::PowerStatus_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::clearpath_base::PowerStatus_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::clearpath_base::PowerStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::clearpath_base::PowerStatus_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::clearpath_base::PowerStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f246c359530c58415aee4fe89d1aca04";
  }

  static const char* value(const ::clearpath_base::PowerStatus_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf246c359530c5841ULL;
  static const uint64_t static_value2 = 0x5aee4fe89d1aca04ULL;
};

template<class ContainerAllocator>
struct DataType< ::clearpath_base::PowerStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "clearpath_base/PowerStatus";
  }

  static const char* value(const ::clearpath_base::PowerStatus_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::clearpath_base::PowerStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"PowerSource[] sources\n"
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
"MSG: clearpath_base/PowerSource\n"
"float32 charge\n"
"int16 capacity\n"
"bool present\n"
"bool in_use\n"
"uint8 description\n"
;
  }

  static const char* value(const ::clearpath_base::PowerStatus_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::clearpath_base::PowerStatus_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.sources);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PowerStatus_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::clearpath_base::PowerStatus_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::clearpath_base::PowerStatus_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "sources[]" << std::endl;
    for (size_t i = 0; i < v.sources.size(); ++i)
    {
      s << indent << "  sources[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::clearpath_base::PowerSource_<ContainerAllocator> >::stream(s, indent + "    ", v.sources[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // CLEARPATH_BASE_MESSAGE_POWERSTATUS_H