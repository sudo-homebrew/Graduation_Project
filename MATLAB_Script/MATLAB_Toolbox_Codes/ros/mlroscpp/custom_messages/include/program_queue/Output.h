// Generated by gencpp from file program_queue/Output.msg
// DO NOT EDIT!


#ifndef PROGRAM_QUEUE_MESSAGE_OUTPUT_H
#define PROGRAM_QUEUE_MESSAGE_OUTPUT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace program_queue
{
template <class ContainerAllocator>
struct Output_
{
  typedef Output_<ContainerAllocator> Type;

  Output_()
    : header()
    , output()  {
    }
  Output_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , output(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _output_type;
  _output_type output;





  typedef boost::shared_ptr< ::program_queue::Output_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::program_queue::Output_<ContainerAllocator> const> ConstPtr;

}; // struct Output_

typedef ::program_queue::Output_<std::allocator<void> > Output;

typedef boost::shared_ptr< ::program_queue::Output > OutputPtr;
typedef boost::shared_ptr< ::program_queue::Output const> OutputConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::program_queue::Output_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::program_queue::Output_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace program_queue

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'program_queue': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/program_queue/msg'], 'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1329247.2/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg', '/mathworks/devel/sbs/31/pmurali.Brobot.j1329247.2/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::program_queue::Output_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::program_queue::Output_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::program_queue::Output_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::program_queue::Output_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::program_queue::Output_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::program_queue::Output_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::program_queue::Output_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c8d7785b7436e847c9dd7124367e2134";
  }

  static const char* value(const ::program_queue::Output_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc8d7785b7436e847ULL;
  static const uint64_t static_value2 = 0xc9dd7124367e2134ULL;
};

template<class ContainerAllocator>
struct DataType< ::program_queue::Output_<ContainerAllocator> >
{
  static const char* value()
  {
    return "program_queue/Output";
  }

  static const char* value(const ::program_queue::Output_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::program_queue::Output_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"string output\n"
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

  static const char* value(const ::program_queue::Output_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::program_queue::Output_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.output);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Output_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::program_queue::Output_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::program_queue::Output_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "output: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.output);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PROGRAM_QUEUE_MESSAGE_OUTPUT_H