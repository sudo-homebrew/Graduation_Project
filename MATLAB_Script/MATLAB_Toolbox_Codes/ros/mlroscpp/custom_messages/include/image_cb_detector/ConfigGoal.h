// Generated by gencpp from file image_cb_detector/ConfigGoal.msg
// DO NOT EDIT!


#ifndef IMAGE_CB_DETECTOR_MESSAGE_CONFIGGOAL_H
#define IMAGE_CB_DETECTOR_MESSAGE_CONFIGGOAL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace image_cb_detector
{
template <class ContainerAllocator>
struct ConfigGoal_
{
  typedef ConfigGoal_<ContainerAllocator> Type;

  ConfigGoal_()
    : num_x(0)
    , num_y(0)
    , spacing_x(0.0)
    , spacing_y(0.0)
    , width_scaling(0.0)
    , height_scaling(0.0)
    , subpixel_window(0)
    , subpixel_zero_zone(0)  {
    }
  ConfigGoal_(const ContainerAllocator& _alloc)
    : num_x(0)
    , num_y(0)
    , spacing_x(0.0)
    , spacing_y(0.0)
    , width_scaling(0.0)
    , height_scaling(0.0)
    , subpixel_window(0)
    , subpixel_zero_zone(0)  {
  (void)_alloc;
    }



   typedef uint32_t _num_x_type;
  _num_x_type num_x;

   typedef uint32_t _num_y_type;
  _num_y_type num_y;

   typedef float _spacing_x_type;
  _spacing_x_type spacing_x;

   typedef float _spacing_y_type;
  _spacing_y_type spacing_y;

   typedef float _width_scaling_type;
  _width_scaling_type width_scaling;

   typedef float _height_scaling_type;
  _height_scaling_type height_scaling;

   typedef uint32_t _subpixel_window_type;
  _subpixel_window_type subpixel_window;

   typedef int32_t _subpixel_zero_zone_type;
  _subpixel_zero_zone_type subpixel_zero_zone;





  typedef boost::shared_ptr< ::image_cb_detector::ConfigGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::image_cb_detector::ConfigGoal_<ContainerAllocator> const> ConstPtr;

}; // struct ConfigGoal_

typedef ::image_cb_detector::ConfigGoal_<std::allocator<void> > ConfigGoal;

typedef boost::shared_ptr< ::image_cb_detector::ConfigGoal > ConfigGoalPtr;
typedef boost::shared_ptr< ::image_cb_detector::ConfigGoal const> ConfigGoalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::image_cb_detector::ConfigGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::image_cb_detector::ConfigGoal_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace image_cb_detector

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/mathworks/devel/sbs/31/hakakarl.Brobot.j1347395/matlab/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg'], 'actionlib': ['/mathworks/devel/sbs/31/hakakarl.Brobot.j1347395/matlab/sys/ros1/glnxa64/ros1/share/actionlib/cmake/../msg'], 'image_cb_detector': ['/local-ssd1/All_Custom_Msgs/PendingPackagesInitial/matlab_msg_gen_ros1/glnxa64/src/image_cb_detector/msg', '/local-ssd1/All_Custom_Msgs/PendingPackagesInitial/matlab_msg_gen_ros1/glnxa64/devel/share/image_cb_detector/msg'], 'std_msgs': ['/mathworks/devel/sbs/31/hakakarl.Brobot.j1347395/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg', '/mathworks/devel/sbs/31/hakakarl.Brobot.j1347395/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/mathworks/devel/sbs/31/hakakarl.Brobot.j1347395/matlab/sys/ros1/glnxa64/ros1/share/actionlib_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::image_cb_detector::ConfigGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::image_cb_detector::ConfigGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::image_cb_detector::ConfigGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::image_cb_detector::ConfigGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::image_cb_detector::ConfigGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::image_cb_detector::ConfigGoal_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::image_cb_detector::ConfigGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fea383eb01c98da472f0666371ce7fb2";
  }

  static const char* value(const ::image_cb_detector::ConfigGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xfea383eb01c98da4ULL;
  static const uint64_t static_value2 = 0x72f0666371ce7fb2ULL;
};

template<class ContainerAllocator>
struct DataType< ::image_cb_detector::ConfigGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "image_cb_detector/ConfigGoal";
  }

  static const char* value(const ::image_cb_detector::ConfigGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::image_cb_detector::ConfigGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"uint32 num_x     # Number of checkerboard corners in the X direction\n"
"uint32 num_y     # Number of corners in the Y direction\n"
"float32 spacing_x  # Spacing between corners in the X direction (meters)\n"
"float32 spacing_y  # Spacing between corners in the Y direction (meters)\n"
"\n"
"# Specify how many times we want to upsample the image.\n"
"#  This is often useful for detecting small checkerboards far away\n"
"float32 width_scaling\n"
"float32 height_scaling\n"
"\n"
"# Configure openCV's subpixel corner detector\n"
"uint32 subpixel_window\n"
"int32  subpixel_zero_zone\n"
"\n"
;
  }

  static const char* value(const ::image_cb_detector::ConfigGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::image_cb_detector::ConfigGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.num_x);
      stream.next(m.num_y);
      stream.next(m.spacing_x);
      stream.next(m.spacing_y);
      stream.next(m.width_scaling);
      stream.next(m.height_scaling);
      stream.next(m.subpixel_window);
      stream.next(m.subpixel_zero_zone);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ConfigGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::image_cb_detector::ConfigGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::image_cb_detector::ConfigGoal_<ContainerAllocator>& v)
  {
    s << indent << "num_x: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.num_x);
    s << indent << "num_y: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.num_y);
    s << indent << "spacing_x: ";
    Printer<float>::stream(s, indent + "  ", v.spacing_x);
    s << indent << "spacing_y: ";
    Printer<float>::stream(s, indent + "  ", v.spacing_y);
    s << indent << "width_scaling: ";
    Printer<float>::stream(s, indent + "  ", v.width_scaling);
    s << indent << "height_scaling: ";
    Printer<float>::stream(s, indent + "  ", v.height_scaling);
    s << indent << "subpixel_window: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.subpixel_window);
    s << indent << "subpixel_zero_zone: ";
    Printer<int32_t>::stream(s, indent + "  ", v.subpixel_zero_zone);
  }
};

} // namespace message_operations
} // namespace ros

#endif // IMAGE_CB_DETECTOR_MESSAGE_CONFIGGOAL_H