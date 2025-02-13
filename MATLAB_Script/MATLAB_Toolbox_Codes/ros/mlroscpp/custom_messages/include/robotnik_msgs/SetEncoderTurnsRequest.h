// Generated by gencpp from file robotnik_msgs/SetEncoderTurnsRequest.msg
// DO NOT EDIT!


#ifndef ROBOTNIK_MSGS_MESSAGE_SETENCODERTURNSREQUEST_H
#define ROBOTNIK_MSGS_MESSAGE_SETENCODERTURNSREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <robotnik_msgs/MotorHeadingOffset.h>

namespace robotnik_msgs
{
template <class ContainerAllocator>
struct SetEncoderTurnsRequest_
{
  typedef SetEncoderTurnsRequest_<ContainerAllocator> Type;

  SetEncoderTurnsRequest_()
    : encoder_turns()  {
    }
  SetEncoderTurnsRequest_(const ContainerAllocator& _alloc)
    : encoder_turns(_alloc)  {
  (void)_alloc;
    }



   typedef  ::robotnik_msgs::MotorHeadingOffset_<ContainerAllocator>  _encoder_turns_type;
  _encoder_turns_type encoder_turns;





  typedef boost::shared_ptr< ::robotnik_msgs::SetEncoderTurnsRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robotnik_msgs::SetEncoderTurnsRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetEncoderTurnsRequest_

typedef ::robotnik_msgs::SetEncoderTurnsRequest_<std::allocator<void> > SetEncoderTurnsRequest;

typedef boost::shared_ptr< ::robotnik_msgs::SetEncoderTurnsRequest > SetEncoderTurnsRequestPtr;
typedef boost::shared_ptr< ::robotnik_msgs::SetEncoderTurnsRequest const> SetEncoderTurnsRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robotnik_msgs::SetEncoderTurnsRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robotnik_msgs::SetEncoderTurnsRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::robotnik_msgs::SetEncoderTurnsRequest_<ContainerAllocator1> & lhs, const ::robotnik_msgs::SetEncoderTurnsRequest_<ContainerAllocator2> & rhs)
{
  return lhs.encoder_turns == rhs.encoder_turns;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::robotnik_msgs::SetEncoderTurnsRequest_<ContainerAllocator1> & lhs, const ::robotnik_msgs::SetEncoderTurnsRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace robotnik_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::robotnik_msgs::SetEncoderTurnsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robotnik_msgs::SetEncoderTurnsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robotnik_msgs::SetEncoderTurnsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robotnik_msgs::SetEncoderTurnsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotnik_msgs::SetEncoderTurnsRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotnik_msgs::SetEncoderTurnsRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robotnik_msgs::SetEncoderTurnsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "34cf70b52bbe3c3e3567eb0d481c62de";
  }

  static const char* value(const ::robotnik_msgs::SetEncoderTurnsRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x34cf70b52bbe3c3eULL;
  static const uint64_t static_value2 = 0x3567eb0d481c62deULL;
};

template<class ContainerAllocator>
struct DataType< ::robotnik_msgs::SetEncoderTurnsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robotnik_msgs/SetEncoderTurnsRequest";
  }

  static const char* value(const ::robotnik_msgs::SetEncoderTurnsRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robotnik_msgs::SetEncoderTurnsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robotnik_msgs/MotorHeadingOffset encoder_turns\n"
"\n"
"================================================================================\n"
"MSG: robotnik_msgs/MotorHeadingOffset\n"
"int32 motor\n"
"float64 value\n"
;
  }

  static const char* value(const ::robotnik_msgs::SetEncoderTurnsRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robotnik_msgs::SetEncoderTurnsRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.encoder_turns);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetEncoderTurnsRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robotnik_msgs::SetEncoderTurnsRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robotnik_msgs::SetEncoderTurnsRequest_<ContainerAllocator>& v)
  {
    s << indent << "encoder_turns: ";
    s << std::endl;
    Printer< ::robotnik_msgs::MotorHeadingOffset_<ContainerAllocator> >::stream(s, indent + "  ", v.encoder_turns);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOTNIK_MSGS_MESSAGE_SETENCODERTURNSREQUEST_H
