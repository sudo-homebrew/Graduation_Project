// Generated by gencpp from file multisense_ros/RawImuData.msg
// DO NOT EDIT!


#ifndef MULTISENSE_ROS_MESSAGE_RAWIMUDATA_H
#define MULTISENSE_ROS_MESSAGE_RAWIMUDATA_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace multisense_ros
{
template <class ContainerAllocator>
struct RawImuData_
{
  typedef RawImuData_<ContainerAllocator> Type;

  RawImuData_()
    : time_stamp()
    , x(0.0)
    , y(0.0)
    , z(0.0)  {
    }
  RawImuData_(const ContainerAllocator& _alloc)
    : time_stamp()
    , x(0.0)
    , y(0.0)
    , z(0.0)  {
  (void)_alloc;
    }



   typedef ros::Time _time_stamp_type;
  _time_stamp_type time_stamp;

   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _z_type;
  _z_type z;





  typedef boost::shared_ptr< ::multisense_ros::RawImuData_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::multisense_ros::RawImuData_<ContainerAllocator> const> ConstPtr;

}; // struct RawImuData_

typedef ::multisense_ros::RawImuData_<std::allocator<void> > RawImuData;

typedef boost::shared_ptr< ::multisense_ros::RawImuData > RawImuDataPtr;
typedef boost::shared_ptr< ::multisense_ros::RawImuData const> RawImuDataConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::multisense_ros::RawImuData_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::multisense_ros::RawImuData_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::multisense_ros::RawImuData_<ContainerAllocator1> & lhs, const ::multisense_ros::RawImuData_<ContainerAllocator2> & rhs)
{
  return lhs.time_stamp == rhs.time_stamp &&
    lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.z == rhs.z;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::multisense_ros::RawImuData_<ContainerAllocator1> & lhs, const ::multisense_ros::RawImuData_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace multisense_ros

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::multisense_ros::RawImuData_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::multisense_ros::RawImuData_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::multisense_ros::RawImuData_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::multisense_ros::RawImuData_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::multisense_ros::RawImuData_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::multisense_ros::RawImuData_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::multisense_ros::RawImuData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bab971dfc7138ffa0d1374504403ac83";
  }

  static const char* value(const ::multisense_ros::RawImuData_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbab971dfc7138ffaULL;
  static const uint64_t static_value2 = 0x0d1374504403ac83ULL;
};

template<class ContainerAllocator>
struct DataType< ::multisense_ros::RawImuData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "multisense_ros/RawImuData";
  }

  static const char* value(const ::multisense_ros::RawImuData_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::multisense_ros::RawImuData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "time    time_stamp\n"
"float32 x\n"
"float32 y\n"
"float32 z\n"
"\n"
;
  }

  static const char* value(const ::multisense_ros::RawImuData_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::multisense_ros::RawImuData_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.time_stamp);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RawImuData_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::multisense_ros::RawImuData_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::multisense_ros::RawImuData_<ContainerAllocator>& v)
  {
    s << indent << "time_stamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.time_stamp);
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<float>::stream(s, indent + "  ", v.z);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MULTISENSE_ROS_MESSAGE_RAWIMUDATA_H