// Generated by gencpp from file roseus/TestName.msg
// DO NOT EDIT!


#ifndef ROSEUS_MESSAGE_TESTNAME_H
#define ROSEUS_MESSAGE_TESTNAME_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <roseus/StringStamped.h>

namespace roseus
{
template <class ContainerAllocator>
struct TestName_
{
  typedef TestName_<ContainerAllocator> Type;

  TestName_()
    : name()  {
    }
  TestName_(const ContainerAllocator& _alloc)
    : name(_alloc)  {
  (void)_alloc;
    }



   typedef  ::roseus::StringStamped_<ContainerAllocator>  _name_type;
  _name_type name;





  typedef boost::shared_ptr< ::roseus::TestName_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roseus::TestName_<ContainerAllocator> const> ConstPtr;

}; // struct TestName_

typedef ::roseus::TestName_<std::allocator<void> > TestName;

typedef boost::shared_ptr< ::roseus::TestName > TestNamePtr;
typedef boost::shared_ptr< ::roseus::TestName const> TestNameConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::roseus::TestName_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::roseus::TestName_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::roseus::TestName_<ContainerAllocator1> & lhs, const ::roseus::TestName_<ContainerAllocator2> & rhs)
{
  return lhs.name == rhs.name;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::roseus::TestName_<ContainerAllocator1> & lhs, const ::roseus::TestName_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace roseus

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::roseus::TestName_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roseus::TestName_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roseus::TestName_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roseus::TestName_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roseus::TestName_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roseus::TestName_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::roseus::TestName_<ContainerAllocator> >
{
  static const char* value()
  {
    return "70bc7fd92cd8428f6a02d7d0df4d9b80";
  }

  static const char* value(const ::roseus::TestName_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x70bc7fd92cd8428fULL;
  static const uint64_t static_value2 = 0x6a02d7d0df4d9b80ULL;
};

template<class ContainerAllocator>
struct DataType< ::roseus::TestName_<ContainerAllocator> >
{
  static const char* value()
  {
    return "roseus/TestName";
  }

  static const char* value(const ::roseus::TestName_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::roseus::TestName_<ContainerAllocator> >
{
  static const char* value()
  {
    return "roseus/StringStamped name\n"
"\n"
"================================================================================\n"
"MSG: roseus/StringStamped\n"
"Header header\n"
"string data\n"
"\n"
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

  static const char* value(const ::roseus::TestName_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::roseus::TestName_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.name);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TestName_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roseus::TestName_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::roseus::TestName_<ContainerAllocator>& v)
  {
    s << indent << "name: ";
    s << std::endl;
    Printer< ::roseus::StringStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.name);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSEUS_MESSAGE_TESTNAME_H