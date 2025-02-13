// Generated by gencpp from file leap_motion/leapros.msg
// DO NOT EDIT!


#ifndef LEAP_MOTION_MESSAGE_LEAPROS_H
#define LEAP_MOTION_MESSAGE_LEAPROS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>

namespace leap_motion
{
template <class ContainerAllocator>
struct leapros_
{
  typedef leapros_<ContainerAllocator> Type;

  leapros_()
    : header()
    , direction()
    , normal()
    , palmpos()
    , ypr()
    , thumb_metacarpal()
    , thumb_proximal()
    , thumb_intermediate()
    , thumb_distal()
    , thumb_tip()
    , index_metacarpal()
    , index_proximal()
    , index_intermediate()
    , index_distal()
    , index_tip()
    , middle_metacarpal()
    , middle_proximal()
    , middle_intermediate()
    , middle_distal()
    , middle_tip()
    , ring_metacarpal()
    , ring_proximal()
    , ring_intermediate()
    , ring_distal()
    , ring_tip()
    , pinky_metacarpal()
    , pinky_proximal()
    , pinky_intermediate()
    , pinky_distal()
    , pinky_tip()  {
    }
  leapros_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , direction(_alloc)
    , normal(_alloc)
    , palmpos(_alloc)
    , ypr(_alloc)
    , thumb_metacarpal(_alloc)
    , thumb_proximal(_alloc)
    , thumb_intermediate(_alloc)
    , thumb_distal(_alloc)
    , thumb_tip(_alloc)
    , index_metacarpal(_alloc)
    , index_proximal(_alloc)
    , index_intermediate(_alloc)
    , index_distal(_alloc)
    , index_tip(_alloc)
    , middle_metacarpal(_alloc)
    , middle_proximal(_alloc)
    , middle_intermediate(_alloc)
    , middle_distal(_alloc)
    , middle_tip(_alloc)
    , ring_metacarpal(_alloc)
    , ring_proximal(_alloc)
    , ring_intermediate(_alloc)
    , ring_distal(_alloc)
    , ring_tip(_alloc)
    , pinky_metacarpal(_alloc)
    , pinky_proximal(_alloc)
    , pinky_intermediate(_alloc)
    , pinky_distal(_alloc)
    , pinky_tip(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _direction_type;
  _direction_type direction;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _normal_type;
  _normal_type normal;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _palmpos_type;
  _palmpos_type palmpos;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _ypr_type;
  _ypr_type ypr;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _thumb_metacarpal_type;
  _thumb_metacarpal_type thumb_metacarpal;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _thumb_proximal_type;
  _thumb_proximal_type thumb_proximal;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _thumb_intermediate_type;
  _thumb_intermediate_type thumb_intermediate;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _thumb_distal_type;
  _thumb_distal_type thumb_distal;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _thumb_tip_type;
  _thumb_tip_type thumb_tip;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _index_metacarpal_type;
  _index_metacarpal_type index_metacarpal;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _index_proximal_type;
  _index_proximal_type index_proximal;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _index_intermediate_type;
  _index_intermediate_type index_intermediate;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _index_distal_type;
  _index_distal_type index_distal;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _index_tip_type;
  _index_tip_type index_tip;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _middle_metacarpal_type;
  _middle_metacarpal_type middle_metacarpal;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _middle_proximal_type;
  _middle_proximal_type middle_proximal;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _middle_intermediate_type;
  _middle_intermediate_type middle_intermediate;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _middle_distal_type;
  _middle_distal_type middle_distal;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _middle_tip_type;
  _middle_tip_type middle_tip;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _ring_metacarpal_type;
  _ring_metacarpal_type ring_metacarpal;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _ring_proximal_type;
  _ring_proximal_type ring_proximal;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _ring_intermediate_type;
  _ring_intermediate_type ring_intermediate;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _ring_distal_type;
  _ring_distal_type ring_distal;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _ring_tip_type;
  _ring_tip_type ring_tip;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _pinky_metacarpal_type;
  _pinky_metacarpal_type pinky_metacarpal;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _pinky_proximal_type;
  _pinky_proximal_type pinky_proximal;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _pinky_intermediate_type;
  _pinky_intermediate_type pinky_intermediate;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _pinky_distal_type;
  _pinky_distal_type pinky_distal;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _pinky_tip_type;
  _pinky_tip_type pinky_tip;





  typedef boost::shared_ptr< ::leap_motion::leapros_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::leap_motion::leapros_<ContainerAllocator> const> ConstPtr;

}; // struct leapros_

typedef ::leap_motion::leapros_<std::allocator<void> > leapros;

typedef boost::shared_ptr< ::leap_motion::leapros > leaprosPtr;
typedef boost::shared_ptr< ::leap_motion::leapros const> leaprosConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::leap_motion::leapros_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::leap_motion::leapros_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::leap_motion::leapros_<ContainerAllocator1> & lhs, const ::leap_motion::leapros_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.direction == rhs.direction &&
    lhs.normal == rhs.normal &&
    lhs.palmpos == rhs.palmpos &&
    lhs.ypr == rhs.ypr &&
    lhs.thumb_metacarpal == rhs.thumb_metacarpal &&
    lhs.thumb_proximal == rhs.thumb_proximal &&
    lhs.thumb_intermediate == rhs.thumb_intermediate &&
    lhs.thumb_distal == rhs.thumb_distal &&
    lhs.thumb_tip == rhs.thumb_tip &&
    lhs.index_metacarpal == rhs.index_metacarpal &&
    lhs.index_proximal == rhs.index_proximal &&
    lhs.index_intermediate == rhs.index_intermediate &&
    lhs.index_distal == rhs.index_distal &&
    lhs.index_tip == rhs.index_tip &&
    lhs.middle_metacarpal == rhs.middle_metacarpal &&
    lhs.middle_proximal == rhs.middle_proximal &&
    lhs.middle_intermediate == rhs.middle_intermediate &&
    lhs.middle_distal == rhs.middle_distal &&
    lhs.middle_tip == rhs.middle_tip &&
    lhs.ring_metacarpal == rhs.ring_metacarpal &&
    lhs.ring_proximal == rhs.ring_proximal &&
    lhs.ring_intermediate == rhs.ring_intermediate &&
    lhs.ring_distal == rhs.ring_distal &&
    lhs.ring_tip == rhs.ring_tip &&
    lhs.pinky_metacarpal == rhs.pinky_metacarpal &&
    lhs.pinky_proximal == rhs.pinky_proximal &&
    lhs.pinky_intermediate == rhs.pinky_intermediate &&
    lhs.pinky_distal == rhs.pinky_distal &&
    lhs.pinky_tip == rhs.pinky_tip;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::leap_motion::leapros_<ContainerAllocator1> & lhs, const ::leap_motion::leapros_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace leap_motion

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::leap_motion::leapros_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::leap_motion::leapros_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::leap_motion::leapros_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::leap_motion::leapros_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::leap_motion::leapros_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::leap_motion::leapros_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::leap_motion::leapros_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e37447f7532c765d6c587f418fd5dd03";
  }

  static const char* value(const ::leap_motion::leapros_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe37447f7532c765dULL;
  static const uint64_t static_value2 = 0x6c587f418fd5dd03ULL;
};

template<class ContainerAllocator>
struct DataType< ::leap_motion::leapros_<ContainerAllocator> >
{
  static const char* value()
  {
    return "leap_motion/leapros";
  }

  static const char* value(const ::leap_motion::leapros_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::leap_motion::leapros_<ContainerAllocator> >
{
  static const char* value()
  {
    return "##################################################\n"
"## Deprecated and will be removed in the future ##\n"
"##################################################\n"
"\n"
"Header header\n"
"geometry_msgs/Vector3 direction\n"
"geometry_msgs/Vector3 normal\n"
"geometry_msgs/Point palmpos\n"
"geometry_msgs/Vector3 ypr\n"
"geometry_msgs/Point thumb_metacarpal\n"
"geometry_msgs/Point thumb_proximal\n"
"geometry_msgs/Point thumb_intermediate\n"
"geometry_msgs/Point thumb_distal\n"
"geometry_msgs/Point thumb_tip\n"
"geometry_msgs/Point index_metacarpal\n"
"geometry_msgs/Point index_proximal\n"
"geometry_msgs/Point index_intermediate\n"
"geometry_msgs/Point index_distal\n"
"geometry_msgs/Point index_tip\n"
"geometry_msgs/Point middle_metacarpal\n"
"geometry_msgs/Point middle_proximal\n"
"geometry_msgs/Point middle_intermediate\n"
"geometry_msgs/Point middle_distal\n"
"geometry_msgs/Point middle_tip\n"
"geometry_msgs/Point ring_metacarpal\n"
"geometry_msgs/Point ring_proximal\n"
"geometry_msgs/Point ring_intermediate\n"
"geometry_msgs/Point ring_distal\n"
"geometry_msgs/Point ring_tip\n"
"geometry_msgs/Point pinky_metacarpal\n"
"geometry_msgs/Point pinky_proximal\n"
"geometry_msgs/Point pinky_intermediate\n"
"geometry_msgs/Point pinky_distal\n"
"geometry_msgs/Point pinky_tip\n"
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
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::leap_motion::leapros_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::leap_motion::leapros_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.direction);
      stream.next(m.normal);
      stream.next(m.palmpos);
      stream.next(m.ypr);
      stream.next(m.thumb_metacarpal);
      stream.next(m.thumb_proximal);
      stream.next(m.thumb_intermediate);
      stream.next(m.thumb_distal);
      stream.next(m.thumb_tip);
      stream.next(m.index_metacarpal);
      stream.next(m.index_proximal);
      stream.next(m.index_intermediate);
      stream.next(m.index_distal);
      stream.next(m.index_tip);
      stream.next(m.middle_metacarpal);
      stream.next(m.middle_proximal);
      stream.next(m.middle_intermediate);
      stream.next(m.middle_distal);
      stream.next(m.middle_tip);
      stream.next(m.ring_metacarpal);
      stream.next(m.ring_proximal);
      stream.next(m.ring_intermediate);
      stream.next(m.ring_distal);
      stream.next(m.ring_tip);
      stream.next(m.pinky_metacarpal);
      stream.next(m.pinky_proximal);
      stream.next(m.pinky_intermediate);
      stream.next(m.pinky_distal);
      stream.next(m.pinky_tip);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct leapros_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::leap_motion::leapros_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::leap_motion::leapros_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "direction: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.direction);
    s << indent << "normal: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.normal);
    s << indent << "palmpos: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.palmpos);
    s << indent << "ypr: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.ypr);
    s << indent << "thumb_metacarpal: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.thumb_metacarpal);
    s << indent << "thumb_proximal: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.thumb_proximal);
    s << indent << "thumb_intermediate: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.thumb_intermediate);
    s << indent << "thumb_distal: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.thumb_distal);
    s << indent << "thumb_tip: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.thumb_tip);
    s << indent << "index_metacarpal: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.index_metacarpal);
    s << indent << "index_proximal: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.index_proximal);
    s << indent << "index_intermediate: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.index_intermediate);
    s << indent << "index_distal: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.index_distal);
    s << indent << "index_tip: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.index_tip);
    s << indent << "middle_metacarpal: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.middle_metacarpal);
    s << indent << "middle_proximal: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.middle_proximal);
    s << indent << "middle_intermediate: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.middle_intermediate);
    s << indent << "middle_distal: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.middle_distal);
    s << indent << "middle_tip: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.middle_tip);
    s << indent << "ring_metacarpal: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.ring_metacarpal);
    s << indent << "ring_proximal: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.ring_proximal);
    s << indent << "ring_intermediate: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.ring_intermediate);
    s << indent << "ring_distal: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.ring_distal);
    s << indent << "ring_tip: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.ring_tip);
    s << indent << "pinky_metacarpal: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.pinky_metacarpal);
    s << indent << "pinky_proximal: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.pinky_proximal);
    s << indent << "pinky_intermediate: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.pinky_intermediate);
    s << indent << "pinky_distal: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.pinky_distal);
    s << indent << "pinky_tip: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.pinky_tip);
  }
};

} // namespace message_operations
} // namespace ros

#endif // LEAP_MOTION_MESSAGE_LEAPROS_H
