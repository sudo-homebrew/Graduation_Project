// Generated by gencpp from file razer_hydra/HydraRaw.msg
// DO NOT EDIT!


#ifndef RAZER_HYDRA_MESSAGE_HYDRARAW_H
#define RAZER_HYDRA_MESSAGE_HYDRARAW_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace razer_hydra
{
template <class ContainerAllocator>
struct HydraRaw_
{
  typedef HydraRaw_<ContainerAllocator> Type;

  HydraRaw_()
    : header()
    , pos()
    , quat()
    , buttons()
    , analog()  {
      pos.assign(0);

      quat.assign(0);

      buttons.assign(0);

      analog.assign(0);
  }
  HydraRaw_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , pos()
    , quat()
    , buttons()
    , analog()  {
  (void)_alloc;
      pos.assign(0);

      quat.assign(0);

      buttons.assign(0);

      analog.assign(0);
  }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef boost::array<int16_t, 6>  _pos_type;
  _pos_type pos;

   typedef boost::array<int16_t, 8>  _quat_type;
  _quat_type quat;

   typedef boost::array<uint8_t, 2>  _buttons_type;
  _buttons_type buttons;

   typedef boost::array<int16_t, 6>  _analog_type;
  _analog_type analog;





  typedef boost::shared_ptr< ::razer_hydra::HydraRaw_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::razer_hydra::HydraRaw_<ContainerAllocator> const> ConstPtr;

}; // struct HydraRaw_

typedef ::razer_hydra::HydraRaw_<std::allocator<void> > HydraRaw;

typedef boost::shared_ptr< ::razer_hydra::HydraRaw > HydraRawPtr;
typedef boost::shared_ptr< ::razer_hydra::HydraRaw const> HydraRawConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::razer_hydra::HydraRaw_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::razer_hydra::HydraRaw_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::razer_hydra::HydraRaw_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::razer_hydra::HydraRaw_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::razer_hydra::HydraRaw_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::razer_hydra::HydraRaw_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::razer_hydra::HydraRaw_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::razer_hydra::HydraRaw_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::razer_hydra::HydraRaw_<ContainerAllocator> >
{
  static const char* value()
  {
    return "abda63674ce89e542bda766f5d8939f7";
  }

  static const char* value(const ::razer_hydra::HydraRaw_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xabda63674ce89e54ULL;
  static const uint64_t static_value2 = 0x2bda766f5d8939f7ULL;
};

template<class ContainerAllocator>
struct DataType< ::razer_hydra::HydraRaw_<ContainerAllocator> >
{
  static const char* value()
  {
    return "razer_hydra/HydraRaw";
  }

  static const char* value(const ::razer_hydra::HydraRaw_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::razer_hydra::HydraRaw_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"int16[6] pos\n"
"int16[8] quat\n"
"uint8[2] buttons\n"
"int16[6] analog\n"
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

  static const char* value(const ::razer_hydra::HydraRaw_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::razer_hydra::HydraRaw_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.pos);
      stream.next(m.quat);
      stream.next(m.buttons);
      stream.next(m.analog);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct HydraRaw_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::razer_hydra::HydraRaw_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::razer_hydra::HydraRaw_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "pos[]" << std::endl;
    for (size_t i = 0; i < v.pos.size(); ++i)
    {
      s << indent << "  pos[" << i << "]: ";
      Printer<int16_t>::stream(s, indent + "  ", v.pos[i]);
    }
    s << indent << "quat[]" << std::endl;
    for (size_t i = 0; i < v.quat.size(); ++i)
    {
      s << indent << "  quat[" << i << "]: ";
      Printer<int16_t>::stream(s, indent + "  ", v.quat[i]);
    }
    s << indent << "buttons[]" << std::endl;
    for (size_t i = 0; i < v.buttons.size(); ++i)
    {
      s << indent << "  buttons[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.buttons[i]);
    }
    s << indent << "analog[]" << std::endl;
    for (size_t i = 0; i < v.analog.size(); ++i)
    {
      s << indent << "  analog[" << i << "]: ";
      Printer<int16_t>::stream(s, indent + "  ", v.analog[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // RAZER_HYDRA_MESSAGE_HYDRARAW_H