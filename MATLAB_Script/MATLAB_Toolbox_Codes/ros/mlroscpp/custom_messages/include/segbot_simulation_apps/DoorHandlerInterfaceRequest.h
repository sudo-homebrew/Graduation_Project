// Generated by gencpp from file segbot_simulation_apps/DoorHandlerInterfaceRequest.msg
// DO NOT EDIT!


#ifndef SEGBOT_SIMULATION_APPS_MESSAGE_DOORHANDLERINTERFACEREQUEST_H
#define SEGBOT_SIMULATION_APPS_MESSAGE_DOORHANDLERINTERFACEREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace segbot_simulation_apps
{
template <class ContainerAllocator>
struct DoorHandlerInterfaceRequest_
{
  typedef DoorHandlerInterfaceRequest_<ContainerAllocator> Type;

  DoorHandlerInterfaceRequest_()
    : door()
    , open(false)
    , all_doors(false)  {
    }
  DoorHandlerInterfaceRequest_(const ContainerAllocator& _alloc)
    : door(_alloc)
    , open(false)
    , all_doors(false)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _door_type;
  _door_type door;

   typedef uint8_t _open_type;
  _open_type open;

   typedef uint8_t _all_doors_type;
  _all_doors_type all_doors;





  typedef boost::shared_ptr< ::segbot_simulation_apps::DoorHandlerInterfaceRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::segbot_simulation_apps::DoorHandlerInterfaceRequest_<ContainerAllocator> const> ConstPtr;

}; // struct DoorHandlerInterfaceRequest_

typedef ::segbot_simulation_apps::DoorHandlerInterfaceRequest_<std::allocator<void> > DoorHandlerInterfaceRequest;

typedef boost::shared_ptr< ::segbot_simulation_apps::DoorHandlerInterfaceRequest > DoorHandlerInterfaceRequestPtr;
typedef boost::shared_ptr< ::segbot_simulation_apps::DoorHandlerInterfaceRequest const> DoorHandlerInterfaceRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::segbot_simulation_apps::DoorHandlerInterfaceRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::segbot_simulation_apps::DoorHandlerInterfaceRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace segbot_simulation_apps

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1339814/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::segbot_simulation_apps::DoorHandlerInterfaceRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::segbot_simulation_apps::DoorHandlerInterfaceRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::segbot_simulation_apps::DoorHandlerInterfaceRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::segbot_simulation_apps::DoorHandlerInterfaceRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::segbot_simulation_apps::DoorHandlerInterfaceRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::segbot_simulation_apps::DoorHandlerInterfaceRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::segbot_simulation_apps::DoorHandlerInterfaceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c8a2e070af7337330d3ec9d7ffc50216";
  }

  static const char* value(const ::segbot_simulation_apps::DoorHandlerInterfaceRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc8a2e070af733733ULL;
  static const uint64_t static_value2 = 0x0d3ec9d7ffc50216ULL;
};

template<class ContainerAllocator>
struct DataType< ::segbot_simulation_apps::DoorHandlerInterfaceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "segbot_simulation_apps/DoorHandlerInterfaceRequest";
  }

  static const char* value(const ::segbot_simulation_apps::DoorHandlerInterfaceRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::segbot_simulation_apps::DoorHandlerInterfaceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string door\n"
"bool open\n"
"bool all_doors\n"
;
  }

  static const char* value(const ::segbot_simulation_apps::DoorHandlerInterfaceRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::segbot_simulation_apps::DoorHandlerInterfaceRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.door);
      stream.next(m.open);
      stream.next(m.all_doors);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DoorHandlerInterfaceRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::segbot_simulation_apps::DoorHandlerInterfaceRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::segbot_simulation_apps::DoorHandlerInterfaceRequest_<ContainerAllocator>& v)
  {
    s << indent << "door: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.door);
    s << indent << "open: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.open);
    s << indent << "all_doors: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.all_doors);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SEGBOT_SIMULATION_APPS_MESSAGE_DOORHANDLERINTERFACEREQUEST_H