// Generated by gencpp from file rocon_app_manager_msgs/StartAppRequest.msg
// DO NOT EDIT!


#ifndef ROCON_APP_MANAGER_MSGS_MESSAGE_STARTAPPREQUEST_H
#define ROCON_APP_MANAGER_MSGS_MESSAGE_STARTAPPREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <rocon_app_manager_msgs/Remapping.h>

namespace rocon_app_manager_msgs
{
template <class ContainerAllocator>
struct StartAppRequest_
{
  typedef StartAppRequest_<ContainerAllocator> Type;

  StartAppRequest_()
    : name()
    , remappings()  {
    }
  StartAppRequest_(const ContainerAllocator& _alloc)
    : name(_alloc)
    , remappings(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  _name_type name;

   typedef std::vector< ::rocon_app_manager_msgs::Remapping_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::rocon_app_manager_msgs::Remapping_<ContainerAllocator> >::other >  _remappings_type;
  _remappings_type remappings;





  typedef boost::shared_ptr< ::rocon_app_manager_msgs::StartAppRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rocon_app_manager_msgs::StartAppRequest_<ContainerAllocator> const> ConstPtr;

}; // struct StartAppRequest_

typedef ::rocon_app_manager_msgs::StartAppRequest_<std::allocator<void> > StartAppRequest;

typedef boost::shared_ptr< ::rocon_app_manager_msgs::StartAppRequest > StartAppRequestPtr;
typedef boost::shared_ptr< ::rocon_app_manager_msgs::StartAppRequest const> StartAppRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rocon_app_manager_msgs::StartAppRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rocon_app_manager_msgs::StartAppRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace rocon_app_manager_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'rocon_app_manager_msgs': ['/mathworks/home/pmurali/Documents/R/matlab_msg_gen_ros1/glnxa64/src/rocon_app_manager_msgs/msg'], 'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1339814/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::rocon_app_manager_msgs::StartAppRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rocon_app_manager_msgs::StartAppRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rocon_app_manager_msgs::StartAppRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rocon_app_manager_msgs::StartAppRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rocon_app_manager_msgs::StartAppRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rocon_app_manager_msgs::StartAppRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rocon_app_manager_msgs::StartAppRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "52a705a1e5933de18edbae79e2aafe49";
  }

  static const char* value(const ::rocon_app_manager_msgs::StartAppRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x52a705a1e5933de1ULL;
  static const uint64_t static_value2 = 0x8edbae79e2aafe49ULL;
};

template<class ContainerAllocator>
struct DataType< ::rocon_app_manager_msgs::StartAppRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rocon_app_manager_msgs/StartAppRequest";
  }

  static const char* value(const ::rocon_app_manager_msgs::StartAppRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rocon_app_manager_msgs::StartAppRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"string name\n"
"Remapping[] remappings\n"
"\n"
"================================================================================\n"
"MSG: rocon_app_manager_msgs/Remapping\n"
"# Describes your typical ros remapping\n"
"\n"
"string remap_from\n"
"string remap_to\n"
;
  }

  static const char* value(const ::rocon_app_manager_msgs::StartAppRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rocon_app_manager_msgs::StartAppRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.name);
      stream.next(m.remappings);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct StartAppRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rocon_app_manager_msgs::StartAppRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rocon_app_manager_msgs::StartAppRequest_<ContainerAllocator>& v)
  {
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
    s << indent << "remappings[]" << std::endl;
    for (size_t i = 0; i < v.remappings.size(); ++i)
    {
      s << indent << "  remappings[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::rocon_app_manager_msgs::Remapping_<ContainerAllocator> >::stream(s, indent + "    ", v.remappings[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROCON_APP_MANAGER_MSGS_MESSAGE_STARTAPPREQUEST_H