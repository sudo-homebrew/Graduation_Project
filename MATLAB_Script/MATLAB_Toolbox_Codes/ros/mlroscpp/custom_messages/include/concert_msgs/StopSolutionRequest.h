// Generated by gencpp from file concert_msgs/StopSolutionRequest.msg
// DO NOT EDIT!


#ifndef CONCERT_MSGS_MESSAGE_STOPSOLUTIONREQUEST_H
#define CONCERT_MSGS_MESSAGE_STOPSOLUTIONREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace concert_msgs
{
template <class ContainerAllocator>
struct StopSolutionRequest_
{
  typedef StopSolutionRequest_<ContainerAllocator> Type;

  StopSolutionRequest_()
    {
    }
  StopSolutionRequest_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::concert_msgs::StopSolutionRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::concert_msgs::StopSolutionRequest_<ContainerAllocator> const> ConstPtr;

}; // struct StopSolutionRequest_

typedef ::concert_msgs::StopSolutionRequest_<std::allocator<void> > StopSolutionRequest;

typedef boost::shared_ptr< ::concert_msgs::StopSolutionRequest > StopSolutionRequestPtr;
typedef boost::shared_ptr< ::concert_msgs::StopSolutionRequest const> StopSolutionRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::concert_msgs::StopSolutionRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::concert_msgs::StopSolutionRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace concert_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'rocon_app_manager_msgs': ['/mathworks/home/pmurali/Documents/Test 3/matlab_msg_gen_ros1/glnxa64/src/rocon_app_manager_msgs/msg'], 'concert_msgs': ['/mathworks/home/pmurali/Documents/Test 3/matlab_msg_gen_ros1/glnxa64/src/concert_msgs/msg'], 'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1339814/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::concert_msgs::StopSolutionRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::concert_msgs::StopSolutionRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::concert_msgs::StopSolutionRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::concert_msgs::StopSolutionRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::concert_msgs::StopSolutionRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::concert_msgs::StopSolutionRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::concert_msgs::StopSolutionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::concert_msgs::StopSolutionRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::concert_msgs::StopSolutionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "concert_msgs/StopSolutionRequest";
  }

  static const char* value(const ::concert_msgs::StopSolutionRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::concert_msgs::StopSolutionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::concert_msgs::StopSolutionRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::concert_msgs::StopSolutionRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct StopSolutionRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::concert_msgs::StopSolutionRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::concert_msgs::StopSolutionRequest_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // CONCERT_MSGS_MESSAGE_STOPSOLUTIONREQUEST_H