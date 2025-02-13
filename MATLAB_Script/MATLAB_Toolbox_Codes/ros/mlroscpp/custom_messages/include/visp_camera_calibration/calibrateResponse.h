// Generated by gencpp from file visp_camera_calibration/calibrateResponse.msg
// DO NOT EDIT!


#ifndef VISP_CAMERA_CALIBRATION_MESSAGE_CALIBRATERESPONSE_H
#define VISP_CAMERA_CALIBRATION_MESSAGE_CALIBRATERESPONSE_H


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
struct calibrateResponse_
{
  typedef calibrateResponse_<ContainerAllocator> Type;

  calibrateResponse_()
    : stdDevErrs()  {
    }
  calibrateResponse_(const ContainerAllocator& _alloc)
    : stdDevErrs(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _stdDevErrs_type;
  _stdDevErrs_type stdDevErrs;





  typedef boost::shared_ptr< ::visp_camera_calibration::calibrateResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::visp_camera_calibration::calibrateResponse_<ContainerAllocator> const> ConstPtr;

}; // struct calibrateResponse_

typedef ::visp_camera_calibration::calibrateResponse_<std::allocator<void> > calibrateResponse;

typedef boost::shared_ptr< ::visp_camera_calibration::calibrateResponse > calibrateResponsePtr;
typedef boost::shared_ptr< ::visp_camera_calibration::calibrateResponse const> calibrateResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::visp_camera_calibration::calibrateResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::visp_camera_calibration::calibrateResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace visp_camera_calibration

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/mathworks/home/pmurali/Documents/S/matlab_msg_gen_ros1/glnxa64/src/sensor_msgs/msg'], 'visp_camera_calibration': ['/mathworks/home/pmurali/Documents/S/matlab_msg_gen_ros1/glnxa64/src/visp_camera_calibration/msg'], 'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1339814/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg', '/mathworks/devel/sbs/31/pmurali.Brobot.j1339814/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1339814/matlab/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::visp_camera_calibration::calibrateResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::visp_camera_calibration::calibrateResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::visp_camera_calibration::calibrateResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::visp_camera_calibration::calibrateResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::visp_camera_calibration::calibrateResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::visp_camera_calibration::calibrateResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::visp_camera_calibration::calibrateResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cd6d27af348dbd9b7530b010497f18b0";
  }

  static const char* value(const ::visp_camera_calibration::calibrateResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcd6d27af348dbd9bULL;
  static const uint64_t static_value2 = 0x7530b010497f18b0ULL;
};

template<class ContainerAllocator>
struct DataType< ::visp_camera_calibration::calibrateResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "visp_camera_calibration/calibrateResponse";
  }

  static const char* value(const ::visp_camera_calibration::calibrateResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::visp_camera_calibration::calibrateResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64[] stdDevErrs\n"
"\n"
;
  }

  static const char* value(const ::visp_camera_calibration::calibrateResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::visp_camera_calibration::calibrateResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.stdDevErrs);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct calibrateResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::visp_camera_calibration::calibrateResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::visp_camera_calibration::calibrateResponse_<ContainerAllocator>& v)
  {
    s << indent << "stdDevErrs[]" << std::endl;
    for (size_t i = 0; i < v.stdDevErrs.size(); ++i)
    {
      s << indent << "  stdDevErrs[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.stdDevErrs[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // VISP_CAMERA_CALIBRATION_MESSAGE_CALIBRATERESPONSE_H
