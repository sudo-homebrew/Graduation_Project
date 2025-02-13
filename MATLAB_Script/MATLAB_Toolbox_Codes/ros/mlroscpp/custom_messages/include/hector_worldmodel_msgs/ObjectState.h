// Generated by gencpp from file hector_worldmodel_msgs/ObjectState.msg
// DO NOT EDIT!


#ifndef HECTOR_WORLDMODEL_MSGS_MESSAGE_OBJECTSTATE_H
#define HECTOR_WORLDMODEL_MSGS_MESSAGE_OBJECTSTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace hector_worldmodel_msgs
{
template <class ContainerAllocator>
struct ObjectState_
{
  typedef ObjectState_<ContainerAllocator> Type;

  ObjectState_()
    : state(0)  {
    }
  ObjectState_(const ContainerAllocator& _alloc)
    : state(0)  {
  (void)_alloc;
    }



   typedef int8_t _state_type;
  _state_type state;



  enum {
    UNKNOWN = 0,
    PENDING = 1,
    ACTIVE = 2,
    INACTIVE = 3,
    CONFIRMED = -1,
    DISCARDED = -2,
    APPROACHING = -3,
  };


  typedef boost::shared_ptr< ::hector_worldmodel_msgs::ObjectState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hector_worldmodel_msgs::ObjectState_<ContainerAllocator> const> ConstPtr;

}; // struct ObjectState_

typedef ::hector_worldmodel_msgs::ObjectState_<std::allocator<void> > ObjectState;

typedef boost::shared_ptr< ::hector_worldmodel_msgs::ObjectState > ObjectStatePtr;
typedef boost::shared_ptr< ::hector_worldmodel_msgs::ObjectState const> ObjectStateConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hector_worldmodel_msgs::ObjectState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hector_worldmodel_msgs::ObjectState_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace hector_worldmodel_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/sensor_msgs/msg'], 'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1329247.2/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg', '/mathworks/devel/sbs/31/pmurali.Brobot.j1329247.2/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'hector_worldmodel_msgs': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/hector_worldmodel_msgs/msg'], 'geometry_msgs': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/geometry_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::hector_worldmodel_msgs::ObjectState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hector_worldmodel_msgs::ObjectState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hector_worldmodel_msgs::ObjectState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hector_worldmodel_msgs::ObjectState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hector_worldmodel_msgs::ObjectState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hector_worldmodel_msgs::ObjectState_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hector_worldmodel_msgs::ObjectState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5bb1b6744a4e40af3e4b8b56b4e06597";
  }

  static const char* value(const ::hector_worldmodel_msgs::ObjectState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5bb1b6744a4e40afULL;
  static const uint64_t static_value2 = 0x3e4b8b56b4e06597ULL;
};

template<class ContainerAllocator>
struct DataType< ::hector_worldmodel_msgs::ObjectState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hector_worldmodel_msgs/ObjectState";
  }

  static const char* value(const ::hector_worldmodel_msgs::ObjectState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hector_worldmodel_msgs::ObjectState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# The state of an object estimate used to track\n"
"# states smaller than 0 disable all updates\n"
"\n"
"# Predefined states. Use states smaller than 0 or bigger than 63 for user defined states.\n"
"int8 UNKNOWN = 0\n"
"int8 PENDING = 1\n"
"int8 ACTIVE  = 2\n"
"int8 INACTIVE = 3\n"
"int8 CONFIRMED = -1\n"
"int8 DISCARDED = -2\n"
"int8 APPROACHING = -3\n"
"\n"
"int8 state\n"
;
  }

  static const char* value(const ::hector_worldmodel_msgs::ObjectState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hector_worldmodel_msgs::ObjectState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.state);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ObjectState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hector_worldmodel_msgs::ObjectState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hector_worldmodel_msgs::ObjectState_<ContainerAllocator>& v)
  {
    s << indent << "state: ";
    Printer<int8_t>::stream(s, indent + "  ", v.state);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HECTOR_WORLDMODEL_MSGS_MESSAGE_OBJECTSTATE_H
