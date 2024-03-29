// Generated by gencpp from file rocon_service_pair_msgs/TestiesResponse.msg
// DO NOT EDIT!


#ifndef ROCON_SERVICE_PAIR_MSGS_MESSAGE_TESTIESRESPONSE_H
#define ROCON_SERVICE_PAIR_MSGS_MESSAGE_TESTIESRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <uuid_msgs/UniqueID.h>

namespace rocon_service_pair_msgs
{
template <class ContainerAllocator>
struct TestiesResponse_
{
  typedef TestiesResponse_<ContainerAllocator> Type;

  TestiesResponse_()
    : id()
    , data()  {
    }
  TestiesResponse_(const ContainerAllocator& _alloc)
    : id(_alloc)
    , data(_alloc)  {
  (void)_alloc;
    }



   typedef  ::uuid_msgs::UniqueID_<ContainerAllocator>  _id_type;
  _id_type id;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _data_type;
  _data_type data;





  typedef boost::shared_ptr< ::rocon_service_pair_msgs::TestiesResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rocon_service_pair_msgs::TestiesResponse_<ContainerAllocator> const> ConstPtr;

}; // struct TestiesResponse_

typedef ::rocon_service_pair_msgs::TestiesResponse_<std::allocator<void> > TestiesResponse;

typedef boost::shared_ptr< ::rocon_service_pair_msgs::TestiesResponse > TestiesResponsePtr;
typedef boost::shared_ptr< ::rocon_service_pair_msgs::TestiesResponse const> TestiesResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rocon_service_pair_msgs::TestiesResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rocon_service_pair_msgs::TestiesResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace rocon_service_pair_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1329247.2/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'rocon_service_pair_msgs': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/rocon_service_pair_msgs/msg'], 'uuid_msgs': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/uuid_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::rocon_service_pair_msgs::TestiesResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rocon_service_pair_msgs::TestiesResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rocon_service_pair_msgs::TestiesResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rocon_service_pair_msgs::TestiesResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rocon_service_pair_msgs::TestiesResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rocon_service_pair_msgs::TestiesResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rocon_service_pair_msgs::TestiesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f2e0bb16a22dc66826bb64ac8b44aeb3";
  }

  static const char* value(const ::rocon_service_pair_msgs::TestiesResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf2e0bb16a22dc668ULL;
  static const uint64_t static_value2 = 0x26bb64ac8b44aeb3ULL;
};

template<class ContainerAllocator>
struct DataType< ::rocon_service_pair_msgs::TestiesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rocon_service_pair_msgs/TestiesResponse";
  }

  static const char* value(const ::rocon_service_pair_msgs::TestiesResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rocon_service_pair_msgs::TestiesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM A SERVICE PAIR DEFINITION ======\n"
"uuid_msgs/UniqueID id\n"
"string data\n"
"\n"
"================================================================================\n"
"MSG: uuid_msgs/UniqueID\n"
"# A universally unique identifier (UUID).\n"
"#\n"
"#  http://en.wikipedia.org/wiki/Universally_unique_identifier\n"
"#  http://tools.ietf.org/html/rfc4122.html\n"
"\n"
"uint8[16] uuid\n"
;
  }

  static const char* value(const ::rocon_service_pair_msgs::TestiesResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rocon_service_pair_msgs::TestiesResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TestiesResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rocon_service_pair_msgs::TestiesResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rocon_service_pair_msgs::TestiesResponse_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    s << std::endl;
    Printer< ::uuid_msgs::UniqueID_<ContainerAllocator> >::stream(s, indent + "  ", v.id);
    s << indent << "data: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.data);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROCON_SERVICE_PAIR_MSGS_MESSAGE_TESTIESRESPONSE_H
