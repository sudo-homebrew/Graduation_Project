// Generated by gencpp from file jsk_footstep_controller/RequireMonitorStatusRequest.msg
// DO NOT EDIT!


#ifndef JSK_FOOTSTEP_CONTROLLER_MESSAGE_REQUIREMONITORSTATUSREQUEST_H
#define JSK_FOOTSTEP_CONTROLLER_MESSAGE_REQUIREMONITORSTATUSREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace jsk_footstep_controller
{
template <class ContainerAllocator>
struct RequireMonitorStatusRequest_
{
  typedef RequireMonitorStatusRequest_<ContainerAllocator> Type;

  RequireMonitorStatusRequest_()
    : header()
    , threshold(0.0)  {
    }
  RequireMonitorStatusRequest_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , threshold(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef float _threshold_type;
  _threshold_type threshold;





  typedef boost::shared_ptr< ::jsk_footstep_controller::RequireMonitorStatusRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_footstep_controller::RequireMonitorStatusRequest_<ContainerAllocator> const> ConstPtr;

}; // struct RequireMonitorStatusRequest_

typedef ::jsk_footstep_controller::RequireMonitorStatusRequest_<std::allocator<void> > RequireMonitorStatusRequest;

typedef boost::shared_ptr< ::jsk_footstep_controller::RequireMonitorStatusRequest > RequireMonitorStatusRequestPtr;
typedef boost::shared_ptr< ::jsk_footstep_controller::RequireMonitorStatusRequest const> RequireMonitorStatusRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_footstep_controller::RequireMonitorStatusRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_footstep_controller::RequireMonitorStatusRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::jsk_footstep_controller::RequireMonitorStatusRequest_<ContainerAllocator1> & lhs, const ::jsk_footstep_controller::RequireMonitorStatusRequest_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.threshold == rhs.threshold;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::jsk_footstep_controller::RequireMonitorStatusRequest_<ContainerAllocator1> & lhs, const ::jsk_footstep_controller::RequireMonitorStatusRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace jsk_footstep_controller

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::jsk_footstep_controller::RequireMonitorStatusRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_footstep_controller::RequireMonitorStatusRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_footstep_controller::RequireMonitorStatusRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_footstep_controller::RequireMonitorStatusRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_footstep_controller::RequireMonitorStatusRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_footstep_controller::RequireMonitorStatusRequest_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_footstep_controller::RequireMonitorStatusRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3dc898e59e7081935abe13d2f48debb8";
  }

  static const char* value(const ::jsk_footstep_controller::RequireMonitorStatusRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3dc898e59e708193ULL;
  static const uint64_t static_value2 = 0x5abe13d2f48debb8ULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_footstep_controller::RequireMonitorStatusRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_footstep_controller/RequireMonitorStatusRequest";
  }

  static const char* value(const ::jsk_footstep_controller::RequireMonitorStatusRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_footstep_controller::RequireMonitorStatusRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"float32 threshold\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::jsk_footstep_controller::RequireMonitorStatusRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_footstep_controller::RequireMonitorStatusRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.threshold);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RequireMonitorStatusRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_footstep_controller::RequireMonitorStatusRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_footstep_controller::RequireMonitorStatusRequest_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "threshold: ";
    Printer<float>::stream(s, indent + "  ", v.threshold);
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_FOOTSTEP_CONTROLLER_MESSAGE_REQUIREMONITORSTATUSREQUEST_H