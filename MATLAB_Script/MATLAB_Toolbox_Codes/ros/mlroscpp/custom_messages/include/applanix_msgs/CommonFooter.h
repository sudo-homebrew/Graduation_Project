// Generated by gencpp from file applanix_msgs/CommonFooter.msg
// DO NOT EDIT!


#ifndef APPLANIX_MSGS_MESSAGE_COMMONFOOTER_H
#define APPLANIX_MSGS_MESSAGE_COMMONFOOTER_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace applanix_msgs
{
template <class ContainerAllocator>
struct CommonFooter_
{
  typedef CommonFooter_<ContainerAllocator> Type;

  CommonFooter_()
    : checksum(0)
    , end()  {
      end.assign(0);
  }
  CommonFooter_(const ContainerAllocator& _alloc)
    : checksum(0)
    , end()  {
  (void)_alloc;
      end.assign(0);
  }



   typedef uint16_t _checksum_type;
  _checksum_type checksum;

   typedef boost::array<uint8_t, 2>  _end_type;
  _end_type end;




  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  END;

  typedef boost::shared_ptr< ::applanix_msgs::CommonFooter_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::applanix_msgs::CommonFooter_<ContainerAllocator> const> ConstPtr;

}; // struct CommonFooter_

typedef ::applanix_msgs::CommonFooter_<std::allocator<void> > CommonFooter;

typedef boost::shared_ptr< ::applanix_msgs::CommonFooter > CommonFooterPtr;
typedef boost::shared_ptr< ::applanix_msgs::CommonFooter const> CommonFooterConstPtr;

// constants requiring out of line definition

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      CommonFooter_<ContainerAllocator>::END =
        
          "$#"
        
        ;
   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::applanix_msgs::CommonFooter_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::applanix_msgs::CommonFooter_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace applanix_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/geometry_msgs/msg'], 'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1329247.2/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'applanix_msgs': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/applanix_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::applanix_msgs::CommonFooter_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::applanix_msgs::CommonFooter_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::applanix_msgs::CommonFooter_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::applanix_msgs::CommonFooter_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::applanix_msgs::CommonFooter_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::applanix_msgs::CommonFooter_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::applanix_msgs::CommonFooter_<ContainerAllocator> >
{
  static const char* value()
  {
    return "603956a77085dd6eb1088e0cade415b7";
  }

  static const char* value(const ::applanix_msgs::CommonFooter_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x603956a77085dd6eULL;
  static const uint64_t static_value2 = 0xb1088e0cade415b7ULL;
};

template<class ContainerAllocator>
struct DataType< ::applanix_msgs::CommonFooter_<ContainerAllocator> >
{
  static const char* value()
  {
    return "applanix_msgs/CommonFooter";
  }

  static const char* value(const ::applanix_msgs::CommonFooter_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::applanix_msgs::CommonFooter_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string END=$#\n"
"uint16 checksum\n"
"uint8[2] end\n"
;
  }

  static const char* value(const ::applanix_msgs::CommonFooter_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::applanix_msgs::CommonFooter_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.checksum);
      stream.next(m.end);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CommonFooter_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::applanix_msgs::CommonFooter_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::applanix_msgs::CommonFooter_<ContainerAllocator>& v)
  {
    s << indent << "checksum: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.checksum);
    s << indent << "end[]" << std::endl;
    for (size_t i = 0; i < v.end.size(); ++i)
    {
      s << indent << "  end[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.end[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // APPLANIX_MSGS_MESSAGE_COMMONFOOTER_H