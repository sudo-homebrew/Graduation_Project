// Generated by gencpp from file pr2_self_test_msgs/TestValue.msg
// DO NOT EDIT!


#ifndef PR2_SELF_TEST_MSGS_MESSAGE_TESTVALUE_H
#define PR2_SELF_TEST_MSGS_MESSAGE_TESTVALUE_H


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
struct TestValue_
{
  typedef TestValue_<ContainerAllocator> Type;

  TestValue_()
    : key()
    , value()
    , min()
    , max()  {
    }
  TestValue_(const ContainerAllocator& _alloc)
    : key(_alloc)
    , value(_alloc)
    , min(_alloc)
    , max(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _key_type;
  _key_type key;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _value_type;
  _value_type value;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _min_type;
  _min_type min;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _max_type;
  _max_type max;





  typedef boost::shared_ptr< ::pr2_self_test_msgs::TestValue_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pr2_self_test_msgs::TestValue_<ContainerAllocator> const> ConstPtr;

}; // struct TestValue_

typedef ::pr2_self_test_msgs::TestValue_<std::allocator<void> > TestValue;

typedef boost::shared_ptr< ::pr2_self_test_msgs::TestValue > TestValuePtr;
typedef boost::shared_ptr< ::pr2_self_test_msgs::TestValue const> TestValueConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pr2_self_test_msgs::TestValue_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pr2_self_test_msgs::TestValue_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace pr2_self_test_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'pr2_self_test_msgs': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/pr2_self_test_msgs/msg'], 'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1329247.2/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::pr2_self_test_msgs::TestValue_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pr2_self_test_msgs::TestValue_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pr2_self_test_msgs::TestValue_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pr2_self_test_msgs::TestValue_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pr2_self_test_msgs::TestValue_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pr2_self_test_msgs::TestValue_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pr2_self_test_msgs::TestValue_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8fffe15af3a2ec4c24d3cf323fdfe721";
  }

  static const char* value(const ::pr2_self_test_msgs::TestValue_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8fffe15af3a2ec4cULL;
  static const uint64_t static_value2 = 0x24d3cf323fdfe721ULL;
};

template<class ContainerAllocator>
struct DataType< ::pr2_self_test_msgs::TestValue_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pr2_self_test_msgs/TestValue";
  }

  static const char* value(const ::pr2_self_test_msgs::TestValue_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pr2_self_test_msgs::TestValue_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# TestValue is recorded value during PR2 qualification\n"
"string key\n"
"string value\n"
"string min\n"
"string max\n"
;
  }

  static const char* value(const ::pr2_self_test_msgs::TestValue_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pr2_self_test_msgs::TestValue_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.key);
      stream.next(m.value);
      stream.next(m.min);
      stream.next(m.max);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TestValue_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pr2_self_test_msgs::TestValue_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pr2_self_test_msgs::TestValue_<ContainerAllocator>& v)
  {
    s << indent << "key: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.key);
    s << indent << "value: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.value);
    s << indent << "min: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.min);
    s << indent << "max: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.max);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PR2_SELF_TEST_MSGS_MESSAGE_TESTVALUE_H