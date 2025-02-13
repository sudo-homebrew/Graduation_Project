// Generated by gencpp from file wge100_camera/BoardConfigRequest.msg
// DO NOT EDIT!


#ifndef WGE100_CAMERA_MESSAGE_BOARDCONFIGREQUEST_H
#define WGE100_CAMERA_MESSAGE_BOARDCONFIGREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace wge100_camera
{
template <class ContainerAllocator>
struct BoardConfigRequest_
{
  typedef BoardConfigRequest_<ContainerAllocator> Type;

  BoardConfigRequest_()
    : serial(0)
    , mac()  {
    }
  BoardConfigRequest_(const ContainerAllocator& _alloc)
    : serial(0)
    , mac(_alloc)  {
  (void)_alloc;
    }



   typedef uint32_t _serial_type;
  _serial_type serial;

   typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  _mac_type;
  _mac_type mac;





  typedef boost::shared_ptr< ::wge100_camera::BoardConfigRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::wge100_camera::BoardConfigRequest_<ContainerAllocator> const> ConstPtr;

}; // struct BoardConfigRequest_

typedef ::wge100_camera::BoardConfigRequest_<std::allocator<void> > BoardConfigRequest;

typedef boost::shared_ptr< ::wge100_camera::BoardConfigRequest > BoardConfigRequestPtr;
typedef boost::shared_ptr< ::wge100_camera::BoardConfigRequest const> BoardConfigRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::wge100_camera::BoardConfigRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::wge100_camera::BoardConfigRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace wge100_camera

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/mathworks/devel/sbs/31/jkonakal.Brobot.j1364265/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::wge100_camera::BoardConfigRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::wge100_camera::BoardConfigRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::wge100_camera::BoardConfigRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::wge100_camera::BoardConfigRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::wge100_camera::BoardConfigRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::wge100_camera::BoardConfigRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::wge100_camera::BoardConfigRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ec9bad54b410ebc79183d761c609dd76";
  }

  static const char* value(const ::wge100_camera::BoardConfigRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xec9bad54b410ebc7ULL;
  static const uint64_t static_value2 = 0x9183d761c609dd76ULL;
};

template<class ContainerAllocator>
struct DataType< ::wge100_camera::BoardConfigRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "wge100_camera/BoardConfigRequest";
  }

  static const char* value(const ::wge100_camera::BoardConfigRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::wge100_camera::BoardConfigRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"\n"
"uint32 serial\n"
"uint8[] mac\n"
;
  }

  static const char* value(const ::wge100_camera::BoardConfigRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::wge100_camera::BoardConfigRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.serial);
      stream.next(m.mac);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct BoardConfigRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::wge100_camera::BoardConfigRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::wge100_camera::BoardConfigRequest_<ContainerAllocator>& v)
  {
    s << indent << "serial: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.serial);
    s << indent << "mac[]" << std::endl;
    for (size_t i = 0; i < v.mac.size(); ++i)
    {
      s << indent << "  mac[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.mac[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // WGE100_CAMERA_MESSAGE_BOARDCONFIGREQUEST_H
