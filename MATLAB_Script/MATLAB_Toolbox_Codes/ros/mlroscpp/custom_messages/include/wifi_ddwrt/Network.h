// Generated by gencpp from file wifi_ddwrt/Network.msg
// DO NOT EDIT!


#ifndef WIFI_DDWRT_MESSAGE_NETWORK_H
#define WIFI_DDWRT_MESSAGE_NETWORK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace wifi_ddwrt
{
template <class ContainerAllocator>
struct Network_
{
  typedef Network_<ContainerAllocator> Type;

  Network_()
    : macattr()
    , essid()
    , channel(0)
    , rssi(0)
    , noise(0)
    , beacon(0)  {
    }
  Network_(const ContainerAllocator& _alloc)
    : macattr(_alloc)
    , essid(_alloc)
    , channel(0)
    , rssi(0)
    , noise(0)
    , beacon(0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _macattr_type;
  _macattr_type macattr;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _essid_type;
  _essid_type essid;

   typedef int32_t _channel_type;
  _channel_type channel;

   typedef int32_t _rssi_type;
  _rssi_type rssi;

   typedef int32_t _noise_type;
  _noise_type noise;

   typedef int32_t _beacon_type;
  _beacon_type beacon;





  typedef boost::shared_ptr< ::wifi_ddwrt::Network_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::wifi_ddwrt::Network_<ContainerAllocator> const> ConstPtr;

}; // struct Network_

typedef ::wifi_ddwrt::Network_<std::allocator<void> > Network;

typedef boost::shared_ptr< ::wifi_ddwrt::Network > NetworkPtr;
typedef boost::shared_ptr< ::wifi_ddwrt::Network const> NetworkConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::wifi_ddwrt::Network_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::wifi_ddwrt::Network_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace wifi_ddwrt

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1329247.2/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg', '/mathworks/devel/sbs/31/pmurali.Brobot.j1329247.2/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'wifi_ddwrt': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/wifi_ddwrt/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::wifi_ddwrt::Network_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::wifi_ddwrt::Network_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::wifi_ddwrt::Network_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::wifi_ddwrt::Network_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::wifi_ddwrt::Network_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::wifi_ddwrt::Network_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::wifi_ddwrt::Network_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b0854419660dc197dd94305843bee07f";
  }

  static const char* value(const ::wifi_ddwrt::Network_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb0854419660dc197ULL;
  static const uint64_t static_value2 = 0xdd94305843bee07fULL;
};

template<class ContainerAllocator>
struct DataType< ::wifi_ddwrt::Network_<ContainerAllocator> >
{
  static const char* value()
  {
    return "wifi_ddwrt/Network";
  }

  static const char* value(const ::wifi_ddwrt::Network_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::wifi_ddwrt::Network_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string macattr\n"
"string essid\n"
"int32 channel\n"
"int32 rssi\n"
"int32 noise\n"
"int32 beacon\n"
;
  }

  static const char* value(const ::wifi_ddwrt::Network_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::wifi_ddwrt::Network_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.macattr);
      stream.next(m.essid);
      stream.next(m.channel);
      stream.next(m.rssi);
      stream.next(m.noise);
      stream.next(m.beacon);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Network_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::wifi_ddwrt::Network_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::wifi_ddwrt::Network_<ContainerAllocator>& v)
  {
    s << indent << "macattr: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.macattr);
    s << indent << "essid: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.essid);
    s << indent << "channel: ";
    Printer<int32_t>::stream(s, indent + "  ", v.channel);
    s << indent << "rssi: ";
    Printer<int32_t>::stream(s, indent + "  ", v.rssi);
    s << indent << "noise: ";
    Printer<int32_t>::stream(s, indent + "  ", v.noise);
    s << indent << "beacon: ";
    Printer<int32_t>::stream(s, indent + "  ", v.beacon);
  }
};

} // namespace message_operations
} // namespace ros

#endif // WIFI_DDWRT_MESSAGE_NETWORK_H