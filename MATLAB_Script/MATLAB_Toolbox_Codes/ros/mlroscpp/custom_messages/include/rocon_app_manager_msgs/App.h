// Generated by gencpp from file rocon_app_manager_msgs/App.msg
// DO NOT EDIT!


#ifndef ROCON_APP_MANAGER_MSGS_MESSAGE_APP_H
#define ROCON_APP_MANAGER_MSGS_MESSAGE_APP_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <rocon_app_manager_msgs/Icon.h>
#include <rocon_app_manager_msgs/PairingClient.h>

namespace rocon_app_manager_msgs
{
template <class ContainerAllocator>
struct App_
{
  typedef App_<ContainerAllocator> Type;

  App_()
    : name()
    , display_name()
    , description()
    , platform()
    , status()
    , icon()
    , pairing_clients()  {
    }
  App_(const ContainerAllocator& _alloc)
    : name(_alloc)
    , display_name(_alloc)
    , description(_alloc)
    , platform(_alloc)
    , status(_alloc)
    , icon(_alloc)
    , pairing_clients(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  _name_type name;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _display_name_type;
  _display_name_type display_name;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _description_type;
  _description_type description;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _platform_type;
  _platform_type platform;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _status_type;
  _status_type status;

   typedef  ::rocon_app_manager_msgs::Icon_<ContainerAllocator>  _icon_type;
  _icon_type icon;

   typedef std::vector< ::rocon_app_manager_msgs::PairingClient_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::rocon_app_manager_msgs::PairingClient_<ContainerAllocator> >::other >  _pairing_clients_type;
  _pairing_clients_type pairing_clients;





  typedef boost::shared_ptr< ::rocon_app_manager_msgs::App_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rocon_app_manager_msgs::App_<ContainerAllocator> const> ConstPtr;

}; // struct App_

typedef ::rocon_app_manager_msgs::App_<std::allocator<void> > App;

typedef boost::shared_ptr< ::rocon_app_manager_msgs::App > AppPtr;
typedef boost::shared_ptr< ::rocon_app_manager_msgs::App const> AppConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rocon_app_manager_msgs::App_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rocon_app_manager_msgs::App_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace rocon_app_manager_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'rocon_app_manager_msgs': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/rocon_app_manager_msgs/msg'], 'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1329247.2/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::rocon_app_manager_msgs::App_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rocon_app_manager_msgs::App_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rocon_app_manager_msgs::App_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rocon_app_manager_msgs::App_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rocon_app_manager_msgs::App_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rocon_app_manager_msgs::App_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rocon_app_manager_msgs::App_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f7c3d8107b83b0a0871d32ad56957836";
  }

  static const char* value(const ::rocon_app_manager_msgs::App_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf7c3d8107b83b0a0ULL;
  static const uint64_t static_value2 = 0x871d32ad56957836ULL;
};

template<class ContainerAllocator>
struct DataType< ::rocon_app_manager_msgs::App_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rocon_app_manager_msgs/App";
  }

  static const char* value(const ::rocon_app_manager_msgs::App_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rocon_app_manager_msgs::App_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# app name\n"
"string name\n"
"# user-friendly display name\n"
"string display_name\n"
"string description\n"
"string platform\n"
"string status\n"
"\n"
"# icon for showing the app\n"
"Icon icon\n"
"#  ordered list (by preference) of pairing clients to interact with this robot app.\n"
"PairingClient[] pairing_clients\n"
"================================================================================\n"
"MSG: rocon_app_manager_msgs/Icon\n"
"# Image data format.  \"jpeg\" or \"png\"\n"
"string format\n"
"\n"
"# Image data.\n"
"uint8[] data\n"
"================================================================================\n"
"MSG: rocon_app_manager_msgs/PairingClient\n"
"# like \"android\" or \"web\" or \"linux\"\n"
"string client_type\n"
"\n"
"# like \"intent = ros.android.teleop\" and \"accelerometer = true\", used to choose which ClientApp to use\n"
"KeyValue[] manager_data\n"
"\n"
"# parameters which just get passed through to the client app.\n"
"KeyValue[] app_data\n"
"================================================================================\n"
"MSG: rocon_app_manager_msgs/KeyValue\n"
"string key\n"
"string value\n"
;
  }

  static const char* value(const ::rocon_app_manager_msgs::App_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rocon_app_manager_msgs::App_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.name);
      stream.next(m.display_name);
      stream.next(m.description);
      stream.next(m.platform);
      stream.next(m.status);
      stream.next(m.icon);
      stream.next(m.pairing_clients);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct App_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rocon_app_manager_msgs::App_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rocon_app_manager_msgs::App_<ContainerAllocator>& v)
  {
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
    s << indent << "display_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.display_name);
    s << indent << "description: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.description);
    s << indent << "platform: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.platform);
    s << indent << "status: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.status);
    s << indent << "icon: ";
    s << std::endl;
    Printer< ::rocon_app_manager_msgs::Icon_<ContainerAllocator> >::stream(s, indent + "  ", v.icon);
    s << indent << "pairing_clients[]" << std::endl;
    for (size_t i = 0; i < v.pairing_clients.size(); ++i)
    {
      s << indent << "  pairing_clients[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::rocon_app_manager_msgs::PairingClient_<ContainerAllocator> >::stream(s, indent + "    ", v.pairing_clients[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROCON_APP_MANAGER_MSGS_MESSAGE_APP_H
