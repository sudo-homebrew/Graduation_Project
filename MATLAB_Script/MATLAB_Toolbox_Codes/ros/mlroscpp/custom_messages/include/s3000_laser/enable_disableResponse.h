// Generated by gencpp from file s3000_laser/enable_disableResponse.msg
// DO NOT EDIT!


#ifndef S3000_LASER_MESSAGE_ENABLE_DISABLERESPONSE_H
#define S3000_LASER_MESSAGE_ENABLE_DISABLERESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace s3000_laser
{
template <class ContainerAllocator>
struct enable_disableResponse_
{
  typedef enable_disableResponse_<ContainerAllocator> Type;

  enable_disableResponse_()
    : ret(false)  {
    }
  enable_disableResponse_(const ContainerAllocator& _alloc)
    : ret(false)  {
  (void)_alloc;
    }



   typedef uint8_t _ret_type;
  _ret_type ret;





  typedef boost::shared_ptr< ::s3000_laser::enable_disableResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::s3000_laser::enable_disableResponse_<ContainerAllocator> const> ConstPtr;

}; // struct enable_disableResponse_

typedef ::s3000_laser::enable_disableResponse_<std::allocator<void> > enable_disableResponse;

typedef boost::shared_ptr< ::s3000_laser::enable_disableResponse > enable_disableResponsePtr;
typedef boost::shared_ptr< ::s3000_laser::enable_disableResponse const> enable_disableResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::s3000_laser::enable_disableResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::s3000_laser::enable_disableResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace s3000_laser

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1339814/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::s3000_laser::enable_disableResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::s3000_laser::enable_disableResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::s3000_laser::enable_disableResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::s3000_laser::enable_disableResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::s3000_laser::enable_disableResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::s3000_laser::enable_disableResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::s3000_laser::enable_disableResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e2cc9e9d8c464550830df49c160979ad";
  }

  static const char* value(const ::s3000_laser::enable_disableResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe2cc9e9d8c464550ULL;
  static const uint64_t static_value2 = 0x830df49c160979adULL;
};

template<class ContainerAllocator>
struct DataType< ::s3000_laser::enable_disableResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "s3000_laser/enable_disableResponse";
  }

  static const char* value(const ::s3000_laser::enable_disableResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::s3000_laser::enable_disableResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool ret\n"
"\n"
;
  }

  static const char* value(const ::s3000_laser::enable_disableResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::s3000_laser::enable_disableResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.ret);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct enable_disableResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::s3000_laser::enable_disableResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::s3000_laser::enable_disableResponse_<ContainerAllocator>& v)
  {
    s << indent << "ret: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.ret);
  }
};

} // namespace message_operations
} // namespace ros

#endif // S3000_LASER_MESSAGE_ENABLE_DISABLERESPONSE_H