// Generated by gencpp from file kingfisher_msgs/Power.msg
// DO NOT EDIT!


#ifndef KINGFISHER_MSGS_MESSAGE_POWER_H
#define KINGFISHER_MSGS_MESSAGE_POWER_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace kingfisher_msgs
{
template <class ContainerAllocator>
struct Power_
{
  typedef Power_<ContainerAllocator> Type;

  Power_()
    : uptime()
    , user_power(0.0)
    , user_power_total(0.0)
    , motor_power_total(0.0)  {
    }
  Power_(const ContainerAllocator& _alloc)
    : uptime()
    , user_power(0.0)
    , user_power_total(0.0)
    , motor_power_total(0.0)  {
  (void)_alloc;
    }



   typedef ros::Duration _uptime_type;
  _uptime_type uptime;

   typedef float _user_power_type;
  _user_power_type user_power;

   typedef float _user_power_total_type;
  _user_power_total_type user_power_total;

   typedef float _motor_power_total_type;
  _motor_power_total_type motor_power_total;





  typedef boost::shared_ptr< ::kingfisher_msgs::Power_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kingfisher_msgs::Power_<ContainerAllocator> const> ConstPtr;

}; // struct Power_

typedef ::kingfisher_msgs::Power_<std::allocator<void> > Power;

typedef boost::shared_ptr< ::kingfisher_msgs::Power > PowerPtr;
typedef boost::shared_ptr< ::kingfisher_msgs::Power const> PowerConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kingfisher_msgs::Power_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kingfisher_msgs::Power_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kingfisher_msgs::Power_<ContainerAllocator1> & lhs, const ::kingfisher_msgs::Power_<ContainerAllocator2> & rhs)
{
  return lhs.uptime == rhs.uptime &&
    lhs.user_power == rhs.user_power &&
    lhs.user_power_total == rhs.user_power_total &&
    lhs.motor_power_total == rhs.motor_power_total;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kingfisher_msgs::Power_<ContainerAllocator1> & lhs, const ::kingfisher_msgs::Power_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kingfisher_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kingfisher_msgs::Power_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kingfisher_msgs::Power_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kingfisher_msgs::Power_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kingfisher_msgs::Power_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kingfisher_msgs::Power_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kingfisher_msgs::Power_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kingfisher_msgs::Power_<ContainerAllocator> >
{
  static const char* value()
  {
    return "389bd7517ed1c204bfed6bdd7ae0dd1d";
  }

  static const char* value(const ::kingfisher_msgs::Power_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x389bd7517ed1c204ULL;
  static const uint64_t static_value2 = 0xbfed6bdd7ae0dd1dULL;
};

template<class ContainerAllocator>
struct DataType< ::kingfisher_msgs::Power_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kingfisher_msgs/Power";
  }

  static const char* value(const ::kingfisher_msgs::Power_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kingfisher_msgs::Power_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Specific power-system data, transmitted by the MCU at 1Hz on /power topic.\n"
"\n"
"duration uptime  # seconds\n"
"\n"
"float32 user_power  # watts\n"
"\n"
"float32 user_power_total  # watt-hours\n"
"float32 motor_power_total  # watt-hours\n"
;
  }

  static const char* value(const ::kingfisher_msgs::Power_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kingfisher_msgs::Power_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.uptime);
      stream.next(m.user_power);
      stream.next(m.user_power_total);
      stream.next(m.motor_power_total);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Power_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kingfisher_msgs::Power_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kingfisher_msgs::Power_<ContainerAllocator>& v)
  {
    s << indent << "uptime: ";
    Printer<ros::Duration>::stream(s, indent + "  ", v.uptime);
    s << indent << "user_power: ";
    Printer<float>::stream(s, indent + "  ", v.user_power);
    s << indent << "user_power_total: ";
    Printer<float>::stream(s, indent + "  ", v.user_power_total);
    s << indent << "motor_power_total: ";
    Printer<float>::stream(s, indent + "  ", v.motor_power_total);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KINGFISHER_MSGS_MESSAGE_POWER_H