// Generated by gencpp from file pr2_self_test_msgs/ScriptDoneRequest.msg
// DO NOT EDIT!


#ifndef PR2_SELF_TEST_MSGS_MESSAGE_SCRIPTDONEREQUEST_H
#define PR2_SELF_TEST_MSGS_MESSAGE_SCRIPTDONEREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace pr2_self_test_msgs
{
template <class ContainerAllocator>
struct ScriptDoneRequest_
{
  typedef ScriptDoneRequest_<ContainerAllocator> Type;

  ScriptDoneRequest_()
    : result(0)
    , failure_msg()  {
    }
  ScriptDoneRequest_(const ContainerAllocator& _alloc)
    : result(0)
    , failure_msg(_alloc)  {
  (void)_alloc;
    }



   typedef int8_t _result_type;
  _result_type result;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _failure_msg_type;
  _failure_msg_type failure_msg;



  enum {
 
    RESULT_OK = 0,
 
    RESULT_FAIL = 1,
 
    RESULT_ERROR = 2,
  };


  typedef boost::shared_ptr< ::pr2_self_test_msgs::ScriptDoneRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pr2_self_test_msgs::ScriptDoneRequest_<ContainerAllocator> const> ConstPtr;

}; // struct ScriptDoneRequest_

typedef ::pr2_self_test_msgs::ScriptDoneRequest_<std::allocator<void> > ScriptDoneRequest;

typedef boost::shared_ptr< ::pr2_self_test_msgs::ScriptDoneRequest > ScriptDoneRequestPtr;
typedef boost::shared_ptr< ::pr2_self_test_msgs::ScriptDoneRequest const> ScriptDoneRequestConstPtr;

// constants requiring out of line definition

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pr2_self_test_msgs::ScriptDoneRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pr2_self_test_msgs::ScriptDoneRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace pr2_self_test_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'pr2_self_test_msgs': ['/mathworks/home/pmurali/Documents/P/matlab_msg_gen_ros1/glnxa64/src/pr2_self_test_msgs/msg'], 'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1339814/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::pr2_self_test_msgs::ScriptDoneRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pr2_self_test_msgs::ScriptDoneRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pr2_self_test_msgs::ScriptDoneRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pr2_self_test_msgs::ScriptDoneRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pr2_self_test_msgs::ScriptDoneRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pr2_self_test_msgs::ScriptDoneRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pr2_self_test_msgs::ScriptDoneRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5ba149050d1d4dafdca40945725022b3";
  }

  static const char* value(const ::pr2_self_test_msgs::ScriptDoneRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5ba149050d1d4dafULL;
  static const uint64_t static_value2 = 0xdca40945725022b3ULL;
};

template<class ContainerAllocator>
struct DataType< ::pr2_self_test_msgs::ScriptDoneRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pr2_self_test_msgs/ScriptDoneRequest";
  }

  static const char* value(const ::pr2_self_test_msgs::ScriptDoneRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pr2_self_test_msgs::ScriptDoneRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "byte RESULT_OK = 0\n"
"byte RESULT_FAIL = 1\n"
"byte RESULT_ERROR = 2\n"
"\n"
"byte result\n"
"string failure_msg\n"
;
  }

  static const char* value(const ::pr2_self_test_msgs::ScriptDoneRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pr2_self_test_msgs::ScriptDoneRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.result);
      stream.next(m.failure_msg);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ScriptDoneRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pr2_self_test_msgs::ScriptDoneRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pr2_self_test_msgs::ScriptDoneRequest_<ContainerAllocator>& v)
  {
    s << indent << "result: ";
    Printer<int8_t>::stream(s, indent + "  ", v.result);
    s << indent << "failure_msg: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.failure_msg);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PR2_SELF_TEST_MSGS_MESSAGE_SCRIPTDONEREQUEST_H