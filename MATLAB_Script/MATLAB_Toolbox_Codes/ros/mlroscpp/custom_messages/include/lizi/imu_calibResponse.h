// Generated by gencpp from file lizi/imu_calibResponse.msg
// DO NOT EDIT!


#ifndef LIZI_MESSAGE_IMU_CALIBRESPONSE_H
#define LIZI_MESSAGE_IMU_CALIBRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace lizi
{
template <class ContainerAllocator>
struct imu_calibResponse_
{
  typedef imu_calibResponse_<ContainerAllocator> Type;

  imu_calibResponse_()
    {
    }
  imu_calibResponse_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::lizi::imu_calibResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::lizi::imu_calibResponse_<ContainerAllocator> const> ConstPtr;

}; // struct imu_calibResponse_

typedef ::lizi::imu_calibResponse_<std::allocator<void> > imu_calibResponse;

typedef boost::shared_ptr< ::lizi::imu_calibResponse > imu_calibResponsePtr;
typedef boost::shared_ptr< ::lizi::imu_calibResponse const> imu_calibResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::lizi::imu_calibResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::lizi::imu_calibResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace lizi

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'lizi': ['/mathworks/home/pmurali/Documents/JKL/matlab_msg_gen_ros1/glnxa64/src/lizi/msg'], 'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1339814/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::lizi::imu_calibResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lizi::imu_calibResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lizi::imu_calibResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lizi::imu_calibResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lizi::imu_calibResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lizi::imu_calibResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::lizi::imu_calibResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::lizi::imu_calibResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::lizi::imu_calibResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "lizi/imu_calibResponse";
  }

  static const char* value(const ::lizi::imu_calibResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::lizi::imu_calibResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"\n"
;
  }

  static const char* value(const ::lizi::imu_calibResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::lizi::imu_calibResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct imu_calibResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::lizi::imu_calibResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::lizi::imu_calibResponse_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // LIZI_MESSAGE_IMU_CALIBRESPONSE_H