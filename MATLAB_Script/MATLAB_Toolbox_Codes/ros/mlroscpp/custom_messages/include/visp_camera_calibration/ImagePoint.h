// Generated by gencpp from file visp_camera_calibration/ImagePoint.msg
// DO NOT EDIT!


#ifndef VISP_CAMERA_CALIBRATION_MESSAGE_IMAGEPOINT_H
#define VISP_CAMERA_CALIBRATION_MESSAGE_IMAGEPOINT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace visp_camera_calibration
{
template <class ContainerAllocator>
struct ImagePoint_
{
  typedef ImagePoint_<ContainerAllocator> Type;

  ImagePoint_()
    : x(0)
    , y(0)  {
    }
  ImagePoint_(const ContainerAllocator& _alloc)
    : x(0)
    , y(0)  {
  (void)_alloc;
    }



   typedef int32_t _x_type;
  _x_type x;

   typedef int32_t _y_type;
  _y_type y;





  typedef boost::shared_ptr< ::visp_camera_calibration::ImagePoint_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::visp_camera_calibration::ImagePoint_<ContainerAllocator> const> ConstPtr;

}; // struct ImagePoint_

typedef ::visp_camera_calibration::ImagePoint_<std::allocator<void> > ImagePoint;

typedef boost::shared_ptr< ::visp_camera_calibration::ImagePoint > ImagePointPtr;
typedef boost::shared_ptr< ::visp_camera_calibration::ImagePoint const> ImagePointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::visp_camera_calibration::ImagePoint_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::visp_camera_calibration::ImagePoint_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace visp_camera_calibration

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/sensor_msgs/msg'], 'visp_camera_calibration': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/visp_camera_calibration/msg'], 'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1329247.2/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg', '/mathworks/devel/sbs/31/pmurali.Brobot.j1329247.2/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/geometry_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::visp_camera_calibration::ImagePoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::visp_camera_calibration::ImagePoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::visp_camera_calibration::ImagePoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::visp_camera_calibration::ImagePoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::visp_camera_calibration::ImagePoint_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::visp_camera_calibration::ImagePoint_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::visp_camera_calibration::ImagePoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bd7b43fd41d4c47bf5c703cc7d016709";
  }

  static const char* value(const ::visp_camera_calibration::ImagePoint_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbd7b43fd41d4c47bULL;
  static const uint64_t static_value2 = 0xf5c703cc7d016709ULL;
};

template<class ContainerAllocator>
struct DataType< ::visp_camera_calibration::ImagePoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "visp_camera_calibration/ImagePoint";
  }

  static const char* value(const ::visp_camera_calibration::ImagePoint_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::visp_camera_calibration::ImagePoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# a point (pixel coordinates) selected in an image\n"
"\n"
"int32 x\n"
"int32 y\n"
;
  }

  static const char* value(const ::visp_camera_calibration::ImagePoint_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::visp_camera_calibration::ImagePoint_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ImagePoint_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::visp_camera_calibration::ImagePoint_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::visp_camera_calibration::ImagePoint_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<int32_t>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<int32_t>::stream(s, indent + "  ", v.y);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VISP_CAMERA_CALIBRATION_MESSAGE_IMAGEPOINT_H