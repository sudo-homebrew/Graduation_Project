// Generated by gencpp from file program_queue/ClearQueueRequest.msg
// DO NOT EDIT!


#ifndef PROGRAM_QUEUE_MESSAGE_CLEARQUEUEREQUEST_H
#define PROGRAM_QUEUE_MESSAGE_CLEARQUEUEREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace program_queue
{
template <class ContainerAllocator>
struct ClearQueueRequest_
{
  typedef ClearQueueRequest_<ContainerAllocator> Type;

  ClearQueueRequest_()
    : token(0)  {
    }
  ClearQueueRequest_(const ContainerAllocator& _alloc)
    : token(0)  {
  (void)_alloc;
    }



   typedef uint64_t _token_type;
  _token_type token;





  typedef boost::shared_ptr< ::program_queue::ClearQueueRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::program_queue::ClearQueueRequest_<ContainerAllocator> const> ConstPtr;

}; // struct ClearQueueRequest_

typedef ::program_queue::ClearQueueRequest_<std::allocator<void> > ClearQueueRequest;

typedef boost::shared_ptr< ::program_queue::ClearQueueRequest > ClearQueueRequestPtr;
typedef boost::shared_ptr< ::program_queue::ClearQueueRequest const> ClearQueueRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::program_queue::ClearQueueRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::program_queue::ClearQueueRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace program_queue

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'program_queue': ['/mathworks/home/pmurali/Documents/P/matlab_msg_gen_ros1/glnxa64/src/program_queue/msg'], 'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1339814/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg', '/mathworks/devel/sbs/31/pmurali.Brobot.j1339814/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::program_queue::ClearQueueRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::program_queue::ClearQueueRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::program_queue::ClearQueueRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::program_queue::ClearQueueRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::program_queue::ClearQueueRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::program_queue::ClearQueueRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::program_queue::ClearQueueRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "14c7152ddd08a9946aaadd642a3c327d";
  }

  static const char* value(const ::program_queue::ClearQueueRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x14c7152ddd08a994ULL;
  static const uint64_t static_value2 = 0x6aaadd642a3c327dULL;
};

template<class ContainerAllocator>
struct DataType< ::program_queue::ClearQueueRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "program_queue/ClearQueueRequest";
  }

  static const char* value(const ::program_queue::ClearQueueRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::program_queue::ClearQueueRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint64 token\n"
;
  }

  static const char* value(const ::program_queue::ClearQueueRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::program_queue::ClearQueueRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.token);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ClearQueueRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::program_queue::ClearQueueRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::program_queue::ClearQueueRequest_<ContainerAllocator>& v)
  {
    s << indent << "token: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.token);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PROGRAM_QUEUE_MESSAGE_CLEARQUEUEREQUEST_H