// Generated by gencpp from file cob_object_detection_msgs/RectArray.msg
// DO NOT EDIT!


#ifndef COB_OBJECT_DETECTION_MSGS_MESSAGE_RECTARRAY_H
#define COB_OBJECT_DETECTION_MSGS_MESSAGE_RECTARRAY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <cob_object_detection_msgs/Rect.h>

namespace cob_object_detection_msgs
{
template <class ContainerAllocator>
struct RectArray_
{
  typedef RectArray_<ContainerAllocator> Type;

  RectArray_()
    : header()
    , rects()  {
    }
  RectArray_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , rects(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::cob_object_detection_msgs::Rect_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::cob_object_detection_msgs::Rect_<ContainerAllocator> >::other >  _rects_type;
  _rects_type rects;





  typedef boost::shared_ptr< ::cob_object_detection_msgs::RectArray_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cob_object_detection_msgs::RectArray_<ContainerAllocator> const> ConstPtr;

}; // struct RectArray_

typedef ::cob_object_detection_msgs::RectArray_<std::allocator<void> > RectArray;

typedef boost::shared_ptr< ::cob_object_detection_msgs::RectArray > RectArrayPtr;
typedef boost::shared_ptr< ::cob_object_detection_msgs::RectArray const> RectArrayConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cob_object_detection_msgs::RectArray_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cob_object_detection_msgs::RectArray_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace cob_object_detection_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/mathworks/devel/sbs/31/hakakarl.Brobot.j1347395/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg', '/mathworks/devel/sbs/31/hakakarl.Brobot.j1347395/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'actionlib': ['/mathworks/devel/sbs/31/hakakarl.Brobot.j1347395/matlab/sys/ros1/glnxa64/ros1/share/actionlib/cmake/../msg'], 'sensor_msgs': ['/mathworks/devel/sbs/31/hakakarl.Brobot.j1347395/matlab/sys/ros1/glnxa64/ros1/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/mathworks/devel/sbs/31/hakakarl.Brobot.j1347395/matlab/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg'], 'cob_object_detection_msgs': ['/local-ssd1/All_Custom_Msgs/PendingPackagesInitial/matlab_msg_gen_ros1/glnxa64/src/cob_object_detection_msgs/msg', '/local-ssd1/All_Custom_Msgs/PendingPackagesInitial/matlab_msg_gen_ros1/glnxa64/devel/share/cob_object_detection_msgs/msg'], 'actionlib_msgs': ['/mathworks/devel/sbs/31/hakakarl.Brobot.j1347395/matlab/sys/ros1/glnxa64/ros1/share/actionlib_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::cob_object_detection_msgs::RectArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cob_object_detection_msgs::RectArray_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cob_object_detection_msgs::RectArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cob_object_detection_msgs::RectArray_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cob_object_detection_msgs::RectArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cob_object_detection_msgs::RectArray_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cob_object_detection_msgs::RectArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e83b38fbaea3a641fa77f009f9bf492e";
  }

  static const char* value(const ::cob_object_detection_msgs::RectArray_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe83b38fbaea3a641ULL;
  static const uint64_t static_value2 = 0xfa77f009f9bf492eULL;
};

template<class ContainerAllocator>
struct DataType< ::cob_object_detection_msgs::RectArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cob_object_detection_msgs/RectArray";
  }

  static const char* value(const ::cob_object_detection_msgs::RectArray_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cob_object_detection_msgs::RectArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"Rect[] rects\n"
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
"MSG: cob_object_detection_msgs/Rect\n"
"int32 x\n"
"int32 y\n"
"int32 width\n"
"int32 height\n"
;
  }

  static const char* value(const ::cob_object_detection_msgs::RectArray_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cob_object_detection_msgs::RectArray_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.rects);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RectArray_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cob_object_detection_msgs::RectArray_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cob_object_detection_msgs::RectArray_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "rects[]" << std::endl;
    for (size_t i = 0; i < v.rects.size(); ++i)
    {
      s << indent << "  rects[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::cob_object_detection_msgs::Rect_<ContainerAllocator> >::stream(s, indent + "    ", v.rects[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // COB_OBJECT_DETECTION_MSGS_MESSAGE_RECTARRAY_H
