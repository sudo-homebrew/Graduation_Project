// Generated by gencpp from file cob_srvs/SetIntResponse.msg
// DO NOT EDIT!


#ifndef COB_SRVS_MESSAGE_SETINTRESPONSE_H
#define COB_SRVS_MESSAGE_SETINTRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace cob_srvs
{
template <class ContainerAllocator>
struct SetIntResponse_
{
  typedef SetIntResponse_<ContainerAllocator> Type;

  SetIntResponse_()
    : success(false)
    , message()  {
    }
  SetIntResponse_(const ContainerAllocator& _alloc)
    : success(false)
    , message(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _message_type;
  _message_type message;





  typedef boost::shared_ptr< ::cob_srvs::SetIntResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cob_srvs::SetIntResponse_<ContainerAllocator> const> ConstPtr;

}; // struct SetIntResponse_

typedef ::cob_srvs::SetIntResponse_<std::allocator<void> > SetIntResponse;

typedef boost::shared_ptr< ::cob_srvs::SetIntResponse > SetIntResponsePtr;
typedef boost::shared_ptr< ::cob_srvs::SetIntResponse const> SetIntResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cob_srvs::SetIntResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cob_srvs::SetIntResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cob_srvs::SetIntResponse_<ContainerAllocator1> & lhs, const ::cob_srvs::SetIntResponse_<ContainerAllocator2> & rhs)
{
  return lhs.success == rhs.success &&
    lhs.message == rhs.message;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cob_srvs::SetIntResponse_<ContainerAllocator1> & lhs, const ::cob_srvs::SetIntResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cob_srvs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::cob_srvs::SetIntResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cob_srvs::SetIntResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cob_srvs::SetIntResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cob_srvs::SetIntResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cob_srvs::SetIntResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cob_srvs::SetIntResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cob_srvs::SetIntResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "937c9679a518e3a18d831e57125ea522";
  }

  static const char* value(const ::cob_srvs::SetIntResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x937c9679a518e3a1ULL;
  static const uint64_t static_value2 = 0x8d831e57125ea522ULL;
};

template<class ContainerAllocator>
struct DataType< ::cob_srvs::SetIntResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cob_srvs/SetIntResponse";
  }

  static const char* value(const ::cob_srvs::SetIntResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cob_srvs::SetIntResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool success\n"
"string message\n"
"\n"
;
  }

  static const char* value(const ::cob_srvs::SetIntResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cob_srvs::SetIntResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
      stream.next(m.message);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetIntResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cob_srvs::SetIntResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cob_srvs::SetIntResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
    s << indent << "message: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.message);
  }
};

} // namespace message_operations
} // namespace ros

#endif // COB_SRVS_MESSAGE_SETINTRESPONSE_H