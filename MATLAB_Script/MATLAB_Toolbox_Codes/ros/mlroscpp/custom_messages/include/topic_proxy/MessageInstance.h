// Generated by gencpp from file topic_proxy/MessageInstance.msg
// DO NOT EDIT!


#ifndef TOPIC_PROXY_MESSAGE_MESSAGEINSTANCE_H
#define TOPIC_PROXY_MESSAGE_MESSAGEINSTANCE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <blob/Blob.h>

namespace topic_proxy
{
template <class ContainerAllocator>
struct MessageInstance_
{
  typedef MessageInstance_<ContainerAllocator> Type;

  MessageInstance_()
    : topic()
    , md5sum()
    , type()
    , message_definition()
    , blob()  {
    }
  MessageInstance_(const ContainerAllocator& _alloc)
    : topic(_alloc)
    , md5sum(_alloc)
    , type(_alloc)
    , message_definition(_alloc)
    , blob(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _topic_type;
  _topic_type topic;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _md5sum_type;
  _md5sum_type md5sum;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _type_type;
  _type_type type;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _message_definition_type;
  _message_definition_type message_definition;

   typedef  ::blob::Blob_<ContainerAllocator>  _blob_type;
  _blob_type blob;





  typedef boost::shared_ptr< ::topic_proxy::MessageInstance_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::topic_proxy::MessageInstance_<ContainerAllocator> const> ConstPtr;

}; // struct MessageInstance_

typedef ::topic_proxy::MessageInstance_<std::allocator<void> > MessageInstance;

typedef boost::shared_ptr< ::topic_proxy::MessageInstance > MessageInstancePtr;
typedef boost::shared_ptr< ::topic_proxy::MessageInstance const> MessageInstanceConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::topic_proxy::MessageInstance_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::topic_proxy::MessageInstance_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace topic_proxy

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'topic_proxy': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/topic_proxy/msg'], 'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1329247.2/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'blob': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/blob/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::topic_proxy::MessageInstance_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::topic_proxy::MessageInstance_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::topic_proxy::MessageInstance_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::topic_proxy::MessageInstance_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::topic_proxy::MessageInstance_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::topic_proxy::MessageInstance_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::topic_proxy::MessageInstance_<ContainerAllocator> >
{
  static const char* value()
  {
    return "69465ef2b8f5727b1913d1e3e2ad35bd";
  }

  static const char* value(const ::topic_proxy::MessageInstance_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x69465ef2b8f5727bULL;
  static const uint64_t static_value2 = 0x1913d1e3e2ad35bdULL;
};

template<class ContainerAllocator>
struct DataType< ::topic_proxy::MessageInstance_<ContainerAllocator> >
{
  static const char* value()
  {
    return "topic_proxy/MessageInstance";
  }

  static const char* value(const ::topic_proxy::MessageInstance_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::topic_proxy::MessageInstance_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string topic\n"
"string md5sum\n"
"string type\n"
"string message_definition\n"
"blob/Blob blob\n"
"\n"
"================================================================================\n"
"MSG: blob/Blob\n"
"bool compressed\n"
"uint8[] data\n"
;
  }

  static const char* value(const ::topic_proxy::MessageInstance_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::topic_proxy::MessageInstance_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.topic);
      stream.next(m.md5sum);
      stream.next(m.type);
      stream.next(m.message_definition);
      stream.next(m.blob);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MessageInstance_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::topic_proxy::MessageInstance_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::topic_proxy::MessageInstance_<ContainerAllocator>& v)
  {
    s << indent << "topic: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.topic);
    s << indent << "md5sum: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.md5sum);
    s << indent << "type: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.type);
    s << indent << "message_definition: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.message_definition);
    s << indent << "blob: ";
    s << std::endl;
    Printer< ::blob::Blob_<ContainerAllocator> >::stream(s, indent + "  ", v.blob);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TOPIC_PROXY_MESSAGE_MESSAGEINSTANCE_H
