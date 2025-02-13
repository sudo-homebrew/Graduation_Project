// Generated by gencpp from file jsk_rviz_plugins/Pictogram.msg
// DO NOT EDIT!


#ifndef JSK_RVIZ_PLUGINS_MESSAGE_PICTOGRAM_H
#define JSK_RVIZ_PLUGINS_MESSAGE_PICTOGRAM_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/ColorRGBA.h>

namespace jsk_rviz_plugins
{
template <class ContainerAllocator>
struct Pictogram_
{
  typedef Pictogram_<ContainerAllocator> Type;

  Pictogram_()
    : header()
    , pose()
    , action(0)
    , mode(0)
    , character()
    , size(0.0)
    , ttl(0.0)
    , speed(0.0)
    , color()  {
    }
  Pictogram_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , pose(_alloc)
    , action(0)
    , mode(0)
    , character(_alloc)
    , size(0.0)
    , ttl(0.0)
    , speed(0.0)
    , color(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef uint8_t _action_type;
  _action_type action;

   typedef uint8_t _mode_type;
  _mode_type mode;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _character_type;
  _character_type character;

   typedef double _size_type;
  _size_type size;

   typedef double _ttl_type;
  _ttl_type ttl;

   typedef double _speed_type;
  _speed_type speed;

   typedef  ::std_msgs::ColorRGBA_<ContainerAllocator>  _color_type;
  _color_type color;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(ADD)
  #undef ADD
#endif
#if defined(_WIN32) && defined(DELETE)
  #undef DELETE
#endif
#if defined(_WIN32) && defined(ROTATE_Z)
  #undef ROTATE_Z
#endif
#if defined(_WIN32) && defined(ROTATE_Y)
  #undef ROTATE_Y
#endif
#if defined(_WIN32) && defined(ROTATE_X)
  #undef ROTATE_X
#endif
#if defined(_WIN32) && defined(JUMP)
  #undef JUMP
#endif
#if defined(_WIN32) && defined(JUMP_ONCE)
  #undef JUMP_ONCE
#endif
#if defined(_WIN32) && defined(PICTOGRAM_MODE)
  #undef PICTOGRAM_MODE
#endif
#if defined(_WIN32) && defined(STRING_MODE)
  #undef STRING_MODE
#endif

  enum {
    ADD = 0u,
    DELETE = 1u,
    ROTATE_Z = 2u,
    ROTATE_Y = 3u,
    ROTATE_X = 4u,
    JUMP = 5u,
    JUMP_ONCE = 6u,
    PICTOGRAM_MODE = 0u,
    STRING_MODE = 1u,
  };


  typedef boost::shared_ptr< ::jsk_rviz_plugins::Pictogram_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_rviz_plugins::Pictogram_<ContainerAllocator> const> ConstPtr;

}; // struct Pictogram_

typedef ::jsk_rviz_plugins::Pictogram_<std::allocator<void> > Pictogram;

typedef boost::shared_ptr< ::jsk_rviz_plugins::Pictogram > PictogramPtr;
typedef boost::shared_ptr< ::jsk_rviz_plugins::Pictogram const> PictogramConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_rviz_plugins::Pictogram_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_rviz_plugins::Pictogram_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::jsk_rviz_plugins::Pictogram_<ContainerAllocator1> & lhs, const ::jsk_rviz_plugins::Pictogram_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.pose == rhs.pose &&
    lhs.action == rhs.action &&
    lhs.mode == rhs.mode &&
    lhs.character == rhs.character &&
    lhs.size == rhs.size &&
    lhs.ttl == rhs.ttl &&
    lhs.speed == rhs.speed &&
    lhs.color == rhs.color;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::jsk_rviz_plugins::Pictogram_<ContainerAllocator1> & lhs, const ::jsk_rviz_plugins::Pictogram_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace jsk_rviz_plugins

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::jsk_rviz_plugins::Pictogram_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_rviz_plugins::Pictogram_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_rviz_plugins::Pictogram_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_rviz_plugins::Pictogram_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_rviz_plugins::Pictogram_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_rviz_plugins::Pictogram_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_rviz_plugins::Pictogram_<ContainerAllocator> >
{
  static const char* value()
  {
    return "29667e5652a8cfdc9c87d2ed97aa7bbc";
  }

  static const char* value(const ::jsk_rviz_plugins::Pictogram_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x29667e5652a8cfdcULL;
  static const uint64_t static_value2 = 0x9c87d2ed97aa7bbcULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_rviz_plugins::Pictogram_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_rviz_plugins/Pictogram";
  }

  static const char* value(const ::jsk_rviz_plugins::Pictogram_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_rviz_plugins::Pictogram_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"geometry_msgs/Pose pose\n"
"uint8 ADD=0\n"
"uint8 DELETE=1\n"
"uint8 ROTATE_Z=2\n"
"uint8 ROTATE_Y=3\n"
"uint8 ROTATE_X=4\n"
"uint8 JUMP=5\n"
"uint8 JUMP_ONCE=6\n"
"uint8 action\n"
"\n"
"uint8 PICTOGRAM_MODE=0 \n"
"uint8 STRING_MODE=1\n"
"\n"
"uint8 mode\n"
"string character\n"
"float64 size\n"
"float64 ttl\n"
"float64 speed\n"
"std_msgs/ColorRGBA color\n"
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
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/ColorRGBA\n"
"float32 r\n"
"float32 g\n"
"float32 b\n"
"float32 a\n"
;
  }

  static const char* value(const ::jsk_rviz_plugins::Pictogram_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_rviz_plugins::Pictogram_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.pose);
      stream.next(m.action);
      stream.next(m.mode);
      stream.next(m.character);
      stream.next(m.size);
      stream.next(m.ttl);
      stream.next(m.speed);
      stream.next(m.color);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Pictogram_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_rviz_plugins::Pictogram_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_rviz_plugins::Pictogram_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "action: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.action);
    s << indent << "mode: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.mode);
    s << indent << "character: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.character);
    s << indent << "size: ";
    Printer<double>::stream(s, indent + "  ", v.size);
    s << indent << "ttl: ";
    Printer<double>::stream(s, indent + "  ", v.ttl);
    s << indent << "speed: ";
    Printer<double>::stream(s, indent + "  ", v.speed);
    s << indent << "color: ";
    s << std::endl;
    Printer< ::std_msgs::ColorRGBA_<ContainerAllocator> >::stream(s, indent + "  ", v.color);
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_RVIZ_PLUGINS_MESSAGE_PICTOGRAM_H
