// Generated by gencpp from file mavros_msgs/FileRenameRequest.msg
// DO NOT EDIT!


#ifndef MAVROS_MSGS_MESSAGE_FILERENAMEREQUEST_H
#define MAVROS_MSGS_MESSAGE_FILERENAMEREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace mavros_msgs
{
template <class ContainerAllocator>
struct FileRenameRequest_
{
  typedef FileRenameRequest_<ContainerAllocator> Type;

  FileRenameRequest_()
    : old_path()
    , new_path()  {
    }
  FileRenameRequest_(const ContainerAllocator& _alloc)
    : old_path(_alloc)
    , new_path(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _old_path_type;
  _old_path_type old_path;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _new_path_type;
  _new_path_type new_path;





  typedef boost::shared_ptr< ::mavros_msgs::FileRenameRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mavros_msgs::FileRenameRequest_<ContainerAllocator> const> ConstPtr;

}; // struct FileRenameRequest_

typedef ::mavros_msgs::FileRenameRequest_<std::allocator<void> > FileRenameRequest;

typedef boost::shared_ptr< ::mavros_msgs::FileRenameRequest > FileRenameRequestPtr;
typedef boost::shared_ptr< ::mavros_msgs::FileRenameRequest const> FileRenameRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mavros_msgs::FileRenameRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mavros_msgs::FileRenameRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace mavros_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'geographic_msgs': ['/mathworks/devel/sbs/33/hakakarl.Brobot.j1694180/matlab/toolbox/ros/mlroscpp/custom_messages/share/geographic_msgs/cmake/../msg'], 'actionlib': ['/mathworks/devel/sbs/33/hakakarl.Brobot.j1694180/matlab/sys/ros1/glnxa64/ros1/share/actionlib/cmake/../msg'], 'std_msgs': ['/mathworks/devel/sbs/33/hakakarl.Brobot.j1694180/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg', '/mathworks/devel/sbs/33/hakakarl.Brobot.j1694180/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'mavros_msgs': ['/local-ssd1/mavros/mavros-1.7.0/mavros-1.7.0/matlab_msg_gen_ros1/glnxa64/src/mavros_msgs/msg'], 'geometry_msgs': ['/mathworks/devel/sbs/33/hakakarl.Brobot.j1694180/matlab/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg'], 'uuid_msgs': ['/mathworks/devel/sbs/33/hakakarl.Brobot.j1694180/matlab/toolbox/ros/mlroscpp/custom_messages/share/uuid_msgs/cmake/../msg'], 'actionlib_msgs': ['/mathworks/devel/sbs/33/hakakarl.Brobot.j1694180/matlab/sys/ros1/glnxa64/ros1/share/actionlib_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::mavros_msgs::FileRenameRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mavros_msgs::FileRenameRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mavros_msgs::FileRenameRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mavros_msgs::FileRenameRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mavros_msgs::FileRenameRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mavros_msgs::FileRenameRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mavros_msgs::FileRenameRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e4a29684c4f7a3290a1bec0a9de2ed01";
  }

  static const char* value(const ::mavros_msgs::FileRenameRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe4a29684c4f7a329ULL;
  static const uint64_t static_value2 = 0x0a1bec0a9de2ed01ULL;
};

template<class ContainerAllocator>
struct DataType< ::mavros_msgs::FileRenameRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mavros_msgs/FileRenameRequest";
  }

  static const char* value(const ::mavros_msgs::FileRenameRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mavros_msgs::FileRenameRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"\n"
"\n"
"\n"
"\n"
"string old_path\n"
"string new_path\n"
;
  }

  static const char* value(const ::mavros_msgs::FileRenameRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mavros_msgs::FileRenameRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.old_path);
      stream.next(m.new_path);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FileRenameRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mavros_msgs::FileRenameRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mavros_msgs::FileRenameRequest_<ContainerAllocator>& v)
  {
    s << indent << "old_path: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.old_path);
    s << indent << "new_path: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.new_path);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MAVROS_MSGS_MESSAGE_FILERENAMEREQUEST_H