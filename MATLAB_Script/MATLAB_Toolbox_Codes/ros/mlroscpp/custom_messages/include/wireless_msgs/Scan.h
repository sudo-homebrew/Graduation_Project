// Generated by gencpp from file wireless_msgs/Scan.msg
// DO NOT EDIT!


#ifndef WIRELESS_MSGS_MESSAGE_SCAN_H
#define WIRELESS_MSGS_MESSAGE_SCAN_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <wireless_msgs/Network.h>

namespace wireless_msgs
{
template <class ContainerAllocator>
struct Scan_
{
  typedef Scan_<ContainerAllocator> Type;

  Scan_()
    : networks()  {
    }
  Scan_(const ContainerAllocator& _alloc)
    : networks(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::wireless_msgs::Network_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::wireless_msgs::Network_<ContainerAllocator> >::other >  _networks_type;
  _networks_type networks;





  typedef boost::shared_ptr< ::wireless_msgs::Scan_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::wireless_msgs::Scan_<ContainerAllocator> const> ConstPtr;

}; // struct Scan_

typedef ::wireless_msgs::Scan_<std::allocator<void> > Scan;

typedef boost::shared_ptr< ::wireless_msgs::Scan > ScanPtr;
typedef boost::shared_ptr< ::wireless_msgs::Scan const> ScanConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::wireless_msgs::Scan_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::wireless_msgs::Scan_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::wireless_msgs::Scan_<ContainerAllocator1> & lhs, const ::wireless_msgs::Scan_<ContainerAllocator2> & rhs)
{
  return lhs.networks == rhs.networks;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::wireless_msgs::Scan_<ContainerAllocator1> & lhs, const ::wireless_msgs::Scan_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace wireless_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::wireless_msgs::Scan_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::wireless_msgs::Scan_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::wireless_msgs::Scan_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::wireless_msgs::Scan_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::wireless_msgs::Scan_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::wireless_msgs::Scan_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::wireless_msgs::Scan_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d32911a086af571ae6871a223a6112f0";
  }

  static const char* value(const ::wireless_msgs::Scan_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd32911a086af571aULL;
  static const uint64_t static_value2 = 0xe6871a223a6112f0ULL;
};

template<class ContainerAllocator>
struct DataType< ::wireless_msgs::Scan_<ContainerAllocator> >
{
  static const char* value()
  {
    return "wireless_msgs/Scan";
  }

  static const char* value(const ::wireless_msgs::Scan_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::wireless_msgs::Scan_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Network[] networks\n"
"\n"
"================================================================================\n"
"MSG: wireless_msgs/Network\n"
"string type\n"
"string essid\n"
"string mac\n"
"string mode\n"
"string frequency\n"
"bool encryption\n"
;
  }

  static const char* value(const ::wireless_msgs::Scan_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::wireless_msgs::Scan_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.networks);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Scan_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::wireless_msgs::Scan_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::wireless_msgs::Scan_<ContainerAllocator>& v)
  {
    s << indent << "networks[]" << std::endl;
    for (size_t i = 0; i < v.networks.size(); ++i)
    {
      s << indent << "  networks[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::wireless_msgs::Network_<ContainerAllocator> >::stream(s, indent + "    ", v.networks[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // WIRELESS_MSGS_MESSAGE_SCAN_H