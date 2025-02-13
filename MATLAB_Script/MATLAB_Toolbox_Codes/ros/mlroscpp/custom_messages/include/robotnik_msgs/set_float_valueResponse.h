// Generated by gencpp from file robotnik_msgs/set_float_valueResponse.msg
// DO NOT EDIT!


#ifndef ROBOTNIK_MSGS_MESSAGE_SET_FLOAT_VALUERESPONSE_H
#define ROBOTNIK_MSGS_MESSAGE_SET_FLOAT_VALUERESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/String.h>

namespace robotnik_msgs
{
template <class ContainerAllocator>
struct set_float_valueResponse_
{
  typedef set_float_valueResponse_<ContainerAllocator> Type;

  set_float_valueResponse_()
    : ret(false)
    , errorMessage()  {
    }
  set_float_valueResponse_(const ContainerAllocator& _alloc)
    : ret(false)
    , errorMessage(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _ret_type;
  _ret_type ret;

   typedef  ::std_msgs::String_<ContainerAllocator>  _errorMessage_type;
  _errorMessage_type errorMessage;





  typedef boost::shared_ptr< ::robotnik_msgs::set_float_valueResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robotnik_msgs::set_float_valueResponse_<ContainerAllocator> const> ConstPtr;

}; // struct set_float_valueResponse_

typedef ::robotnik_msgs::set_float_valueResponse_<std::allocator<void> > set_float_valueResponse;

typedef boost::shared_ptr< ::robotnik_msgs::set_float_valueResponse > set_float_valueResponsePtr;
typedef boost::shared_ptr< ::robotnik_msgs::set_float_valueResponse const> set_float_valueResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robotnik_msgs::set_float_valueResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robotnik_msgs::set_float_valueResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::robotnik_msgs::set_float_valueResponse_<ContainerAllocator1> & lhs, const ::robotnik_msgs::set_float_valueResponse_<ContainerAllocator2> & rhs)
{
  return lhs.ret == rhs.ret &&
    lhs.errorMessage == rhs.errorMessage;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::robotnik_msgs::set_float_valueResponse_<ContainerAllocator1> & lhs, const ::robotnik_msgs::set_float_valueResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace robotnik_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::robotnik_msgs::set_float_valueResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robotnik_msgs::set_float_valueResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robotnik_msgs::set_float_valueResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robotnik_msgs::set_float_valueResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotnik_msgs::set_float_valueResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotnik_msgs::set_float_valueResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robotnik_msgs::set_float_valueResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b607fdf6f13faab17a8c316347e7f65b";
  }

  static const char* value(const ::robotnik_msgs::set_float_valueResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb607fdf6f13faab1ULL;
  static const uint64_t static_value2 = 0x7a8c316347e7f65bULL;
};

template<class ContainerAllocator>
struct DataType< ::robotnik_msgs::set_float_valueResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robotnik_msgs/set_float_valueResponse";
  }

  static const char* value(const ::robotnik_msgs::set_float_valueResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robotnik_msgs::set_float_valueResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool ret\n"
"std_msgs/String errorMessage\n"
"\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/String\n"
"string data\n"
;
  }

  static const char* value(const ::robotnik_msgs::set_float_valueResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robotnik_msgs::set_float_valueResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.ret);
      stream.next(m.errorMessage);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct set_float_valueResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robotnik_msgs::set_float_valueResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robotnik_msgs::set_float_valueResponse_<ContainerAllocator>& v)
  {
    s << indent << "ret: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.ret);
    s << indent << "errorMessage: ";
    s << std::endl;
    Printer< ::std_msgs::String_<ContainerAllocator> >::stream(s, indent + "  ", v.errorMessage);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOTNIK_MSGS_MESSAGE_SET_FLOAT_VALUERESPONSE_H
