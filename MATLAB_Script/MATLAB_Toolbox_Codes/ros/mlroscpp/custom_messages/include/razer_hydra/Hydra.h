// Generated by gencpp from file razer_hydra/Hydra.msg
// DO NOT EDIT!


#ifndef RAZER_HYDRA_MESSAGE_HYDRA_H
#define RAZER_HYDRA_MESSAGE_HYDRA_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <razer_hydra/HydraPaddle.h>

namespace razer_hydra
{
template <class ContainerAllocator>
struct Hydra_
{
  typedef Hydra_<ContainerAllocator> Type;

  Hydra_()
    : header()
    , paddles()  {
    }
  Hydra_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , paddles()  {
  (void)_alloc;
      paddles.assign( ::razer_hydra::HydraPaddle_<ContainerAllocator> (_alloc));
  }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef boost::array< ::razer_hydra::HydraPaddle_<ContainerAllocator> , 2>  _paddles_type;
  _paddles_type paddles;



  enum {
    LEFT = 0u,
    RIGHT = 1u,
  };


  typedef boost::shared_ptr< ::razer_hydra::Hydra_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::razer_hydra::Hydra_<ContainerAllocator> const> ConstPtr;

}; // struct Hydra_

typedef ::razer_hydra::Hydra_<std::allocator<void> > Hydra;

typedef boost::shared_ptr< ::razer_hydra::Hydra > HydraPtr;
typedef boost::shared_ptr< ::razer_hydra::Hydra const> HydraConstPtr;

// constants requiring out of line definition

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::razer_hydra::Hydra_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::razer_hydra::Hydra_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace razer_hydra

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'razer_hydra': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/razer_hydra/msg'], 'geometry_msgs': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/geometry_msgs/msg'], 'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1329247.2/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg', '/mathworks/devel/sbs/31/pmurali.Brobot.j1329247.2/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::razer_hydra::Hydra_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::razer_hydra::Hydra_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::razer_hydra::Hydra_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::razer_hydra::Hydra_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::razer_hydra::Hydra_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::razer_hydra::Hydra_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::razer_hydra::Hydra_<ContainerAllocator> >
{
  static const char* value()
  {
    return "63e785fc661607e9f6f13322700f70b8";
  }

  static const char* value(const ::razer_hydra::Hydra_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x63e785fc661607e9ULL;
  static const uint64_t static_value2 = 0xf6f13322700f70b8ULL;
};

template<class ContainerAllocator>
struct DataType< ::razer_hydra::Hydra_<ContainerAllocator> >
{
  static const char* value()
  {
    return "razer_hydra/Hydra";
  }

  static const char* value(const ::razer_hydra::Hydra_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::razer_hydra::Hydra_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"HydraPaddle[2] paddles\n"
"\n"
"uint8 LEFT = 0\n"
"uint8 RIGHT = 1\n"
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
"\n"
"================================================================================\n"
"MSG: razer_hydra/HydraPaddle\n"
"geometry_msgs/Transform transform\n"
"bool[7] buttons\n"
"float32[2] joy\n"
"float32 trigger\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Transform\n"
"# This represents the transform between two coordinate frames in free space.\n"
"\n"
"Vector3 translation\n"
"Quaternion rotation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::razer_hydra::Hydra_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::razer_hydra::Hydra_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.paddles);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Hydra_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::razer_hydra::Hydra_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::razer_hydra::Hydra_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "paddles[]" << std::endl;
    for (size_t i = 0; i < v.paddles.size(); ++i)
    {
      s << indent << "  paddles[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::razer_hydra::HydraPaddle_<ContainerAllocator> >::stream(s, indent + "    ", v.paddles[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // RAZER_HYDRA_MESSAGE_HYDRA_H