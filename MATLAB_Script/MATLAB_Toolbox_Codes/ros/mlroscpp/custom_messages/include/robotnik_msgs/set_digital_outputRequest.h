// Generated by gencpp from file robotnik_msgs/set_digital_outputRequest.msg
// DO NOT EDIT!


#ifndef ROBOTNIK_MSGS_MESSAGE_SET_DIGITAL_OUTPUTREQUEST_H
#define ROBOTNIK_MSGS_MESSAGE_SET_DIGITAL_OUTPUTREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace robotnik_msgs
{
template <class ContainerAllocator>
struct set_digital_outputRequest_
{
  typedef set_digital_outputRequest_<ContainerAllocator> Type;

  set_digital_outputRequest_()
    : output(0)
    , value(false)  {
    }
  set_digital_outputRequest_(const ContainerAllocator& _alloc)
    : output(0)
    , value(false)  {
  (void)_alloc;
    }



   typedef int8_t _output_type;
  _output_type output;

   typedef uint8_t _value_type;
  _value_type value;





  typedef boost::shared_ptr< ::robotnik_msgs::set_digital_outputRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robotnik_msgs::set_digital_outputRequest_<ContainerAllocator> const> ConstPtr;

}; // struct set_digital_outputRequest_

typedef ::robotnik_msgs::set_digital_outputRequest_<std::allocator<void> > set_digital_outputRequest;

typedef boost::shared_ptr< ::robotnik_msgs::set_digital_outputRequest > set_digital_outputRequestPtr;
typedef boost::shared_ptr< ::robotnik_msgs::set_digital_outputRequest const> set_digital_outputRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robotnik_msgs::set_digital_outputRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robotnik_msgs::set_digital_outputRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::robotnik_msgs::set_digital_outputRequest_<ContainerAllocator1> & lhs, const ::robotnik_msgs::set_digital_outputRequest_<ContainerAllocator2> & rhs)
{
  return lhs.output == rhs.output &&
    lhs.value == rhs.value;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::robotnik_msgs::set_digital_outputRequest_<ContainerAllocator1> & lhs, const ::robotnik_msgs::set_digital_outputRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace robotnik_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::robotnik_msgs::set_digital_outputRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robotnik_msgs::set_digital_outputRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robotnik_msgs::set_digital_outputRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robotnik_msgs::set_digital_outputRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotnik_msgs::set_digital_outputRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotnik_msgs::set_digital_outputRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robotnik_msgs::set_digital_outputRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e1dc4e43688f0a8d32bc10e72fd189d2";
  }

  static const char* value(const ::robotnik_msgs::set_digital_outputRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe1dc4e43688f0a8dULL;
  static const uint64_t static_value2 = 0x32bc10e72fd189d2ULL;
};

template<class ContainerAllocator>
struct DataType< ::robotnik_msgs::set_digital_outputRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robotnik_msgs/set_digital_outputRequest";
  }

  static const char* value(const ::robotnik_msgs::set_digital_outputRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robotnik_msgs::set_digital_outputRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int8 output\n"
"bool value\n"
;
  }

  static const char* value(const ::robotnik_msgs::set_digital_outputRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robotnik_msgs::set_digital_outputRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.output);
      stream.next(m.value);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct set_digital_outputRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robotnik_msgs::set_digital_outputRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robotnik_msgs::set_digital_outputRequest_<ContainerAllocator>& v)
  {
    s << indent << "output: ";
    Printer<int8_t>::stream(s, indent + "  ", v.output);
    s << indent << "value: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.value);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOTNIK_MSGS_MESSAGE_SET_DIGITAL_OUTPUTREQUEST_H
