// Generated by gencpp from file cob_trajectory_controller/SetFloatRequest.msg
// DO NOT EDIT!


#ifndef COB_TRAJECTORY_CONTROLLER_MESSAGE_SETFLOATREQUEST_H
#define COB_TRAJECTORY_CONTROLLER_MESSAGE_SETFLOATREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Float64.h>

namespace cob_trajectory_controller
{
template <class ContainerAllocator>
struct SetFloatRequest_
{
  typedef SetFloatRequest_<ContainerAllocator> Type;

  SetFloatRequest_()
    : value()  {
    }
  SetFloatRequest_(const ContainerAllocator& _alloc)
    : value(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Float64_<ContainerAllocator>  _value_type;
  _value_type value;





  typedef boost::shared_ptr< ::cob_trajectory_controller::SetFloatRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cob_trajectory_controller::SetFloatRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetFloatRequest_

typedef ::cob_trajectory_controller::SetFloatRequest_<std::allocator<void> > SetFloatRequest;

typedef boost::shared_ptr< ::cob_trajectory_controller::SetFloatRequest > SetFloatRequestPtr;
typedef boost::shared_ptr< ::cob_trajectory_controller::SetFloatRequest const> SetFloatRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cob_trajectory_controller::SetFloatRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cob_trajectory_controller::SetFloatRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace cob_trajectory_controller

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1339814/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg', '/mathworks/devel/sbs/31/pmurali.Brobot.j1339814/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::cob_trajectory_controller::SetFloatRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cob_trajectory_controller::SetFloatRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cob_trajectory_controller::SetFloatRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cob_trajectory_controller::SetFloatRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cob_trajectory_controller::SetFloatRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cob_trajectory_controller::SetFloatRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cob_trajectory_controller::SetFloatRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1b2e6c76ff0a23e68a43b77651f66f18";
  }

  static const char* value(const ::cob_trajectory_controller::SetFloatRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1b2e6c76ff0a23e6ULL;
  static const uint64_t static_value2 = 0x8a43b77651f66f18ULL;
};

template<class ContainerAllocator>
struct DataType< ::cob_trajectory_controller::SetFloatRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cob_trajectory_controller/SetFloatRequest";
  }

  static const char* value(const ::cob_trajectory_controller::SetFloatRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cob_trajectory_controller::SetFloatRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Float64 value\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Float64\n"
"float64 data\n"
;
  }

  static const char* value(const ::cob_trajectory_controller::SetFloatRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cob_trajectory_controller::SetFloatRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.value);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetFloatRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cob_trajectory_controller::SetFloatRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cob_trajectory_controller::SetFloatRequest_<ContainerAllocator>& v)
  {
    s << indent << "value: ";
    s << std::endl;
    Printer< ::std_msgs::Float64_<ContainerAllocator> >::stream(s, indent + "  ", v.value);
  }
};

} // namespace message_operations
} // namespace ros

#endif // COB_TRAJECTORY_CONTROLLER_MESSAGE_SETFLOATREQUEST_H