// Generated by gencpp from file fkie_multimaster_msgs/TaskResponse.msg
// DO NOT EDIT!


#ifndef FKIE_MULTIMASTER_MSGS_MESSAGE_TASKRESPONSE_H
#define FKIE_MULTIMASTER_MSGS_MESSAGE_TASKRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace fkie_multimaster_msgs
{
template <class ContainerAllocator>
struct TaskResponse_
{
  typedef TaskResponse_<ContainerAllocator> Type;

  TaskResponse_()
    {
    }
  TaskResponse_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::fkie_multimaster_msgs::TaskResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::fkie_multimaster_msgs::TaskResponse_<ContainerAllocator> const> ConstPtr;

}; // struct TaskResponse_

typedef ::fkie_multimaster_msgs::TaskResponse_<std::allocator<void> > TaskResponse;

typedef boost::shared_ptr< ::fkie_multimaster_msgs::TaskResponse > TaskResponsePtr;
typedef boost::shared_ptr< ::fkie_multimaster_msgs::TaskResponse const> TaskResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::fkie_multimaster_msgs::TaskResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::fkie_multimaster_msgs::TaskResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace fkie_multimaster_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::fkie_multimaster_msgs::TaskResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::fkie_multimaster_msgs::TaskResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::fkie_multimaster_msgs::TaskResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::fkie_multimaster_msgs::TaskResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::fkie_multimaster_msgs::TaskResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::fkie_multimaster_msgs::TaskResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::fkie_multimaster_msgs::TaskResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::fkie_multimaster_msgs::TaskResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::fkie_multimaster_msgs::TaskResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fkie_multimaster_msgs/TaskResponse";
  }

  static const char* value(const ::fkie_multimaster_msgs::TaskResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::fkie_multimaster_msgs::TaskResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::fkie_multimaster_msgs::TaskResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::fkie_multimaster_msgs::TaskResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TaskResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::fkie_multimaster_msgs::TaskResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::fkie_multimaster_msgs::TaskResponse_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // FKIE_MULTIMASTER_MSGS_MESSAGE_TASKRESPONSE_H
