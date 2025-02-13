// Generated by gencpp from file multimaster_msgs_fkie/MasterState.msg
// DO NOT EDIT!


#ifndef MULTIMASTER_MSGS_FKIE_MESSAGE_MASTERSTATE_H
#define MULTIMASTER_MSGS_FKIE_MESSAGE_MASTERSTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <multimaster_msgs_fkie/ROSMaster.h>

namespace multimaster_msgs_fkie
{
template <class ContainerAllocator>
struct MasterState_
{
  typedef MasterState_<ContainerAllocator> Type;

  MasterState_()
    : state()
    , master()  {
    }
  MasterState_(const ContainerAllocator& _alloc)
    : state(_alloc)
    , master(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _state_type;
  _state_type state;

   typedef  ::multimaster_msgs_fkie::ROSMaster_<ContainerAllocator>  _master_type;
  _master_type master;




  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  STATE_NEW;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  STATE_REMOVED;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  STATE_CHANGED;

  typedef boost::shared_ptr< ::multimaster_msgs_fkie::MasterState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::multimaster_msgs_fkie::MasterState_<ContainerAllocator> const> ConstPtr;

}; // struct MasterState_

typedef ::multimaster_msgs_fkie::MasterState_<std::allocator<void> > MasterState;

typedef boost::shared_ptr< ::multimaster_msgs_fkie::MasterState > MasterStatePtr;
typedef boost::shared_ptr< ::multimaster_msgs_fkie::MasterState const> MasterStateConstPtr;

// constants requiring out of line definition

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      MasterState_<ContainerAllocator>::STATE_NEW =
        
          "'new'"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      MasterState_<ContainerAllocator>::STATE_REMOVED =
        
          "'removed'"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      MasterState_<ContainerAllocator>::STATE_CHANGED =
        
          "'changed'"
        
        ;
   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::multimaster_msgs_fkie::MasterState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::multimaster_msgs_fkie::MasterState_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace multimaster_msgs_fkie

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'multimaster_msgs_fkie': ['/mathworks/home/pmurali/Documents/rosjava_messages/GEN-4-6/11/matlab_msg_gen_ros1/glnxa64/src/multimaster_msgs_fkie/msg'], 'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1352567/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg', '/mathworks/devel/sbs/31/pmurali.Brobot.j1352567/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::multimaster_msgs_fkie::MasterState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::multimaster_msgs_fkie::MasterState_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::multimaster_msgs_fkie::MasterState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::multimaster_msgs_fkie::MasterState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::multimaster_msgs_fkie::MasterState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::multimaster_msgs_fkie::MasterState_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::multimaster_msgs_fkie::MasterState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "80884072ff659ac99555a763e9ba0b23";
  }

  static const char* value(const ::multimaster_msgs_fkie::MasterState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x80884072ff659ac9ULL;
  static const uint64_t static_value2 = 0x9555a763e9ba0b23ULL;
};

template<class ContainerAllocator>
struct DataType< ::multimaster_msgs_fkie::MasterState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "multimaster_msgs_fkie/MasterState";
  }

  static const char* value(const ::multimaster_msgs_fkie::MasterState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::multimaster_msgs_fkie::MasterState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string state\n"
"multimaster_msgs_fkie/ROSMaster master\n"
"\n"
"string STATE_NEW='new'\n"
"string STATE_REMOVED='removed'\n"
"string STATE_CHANGED='changed'\n"
"\n"
"================================================================================\n"
"MSG: multimaster_msgs_fkie/ROSMaster\n"
"string name\n"
"# ROS Master URI\n"
"string uri\n"
"# The timestamp of the state of the remoter ROS master\n"
"float64 timestamp\n"
"# The timestamp of the state of the remoter ROS master, without the changes maked while a synchronization.\n"
"float64 timestamp_local\n"
"bool online\n"
"string discoverer_name\n"
"string monitoruri\n"
;
  }

  static const char* value(const ::multimaster_msgs_fkie::MasterState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::multimaster_msgs_fkie::MasterState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.state);
      stream.next(m.master);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MasterState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::multimaster_msgs_fkie::MasterState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::multimaster_msgs_fkie::MasterState_<ContainerAllocator>& v)
  {
    s << indent << "state: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.state);
    s << indent << "master: ";
    s << std::endl;
    Printer< ::multimaster_msgs_fkie::ROSMaster_<ContainerAllocator> >::stream(s, indent + "  ", v.master);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MULTIMASTER_MSGS_FKIE_MESSAGE_MASTERSTATE_H
