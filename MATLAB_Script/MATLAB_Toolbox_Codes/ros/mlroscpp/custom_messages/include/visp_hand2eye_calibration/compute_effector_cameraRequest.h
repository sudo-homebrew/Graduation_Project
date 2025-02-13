// Generated by gencpp from file visp_hand2eye_calibration/compute_effector_cameraRequest.msg
// DO NOT EDIT!


#ifndef VISP_HAND2EYE_CALIBRATION_MESSAGE_COMPUTE_EFFECTOR_CAMERAREQUEST_H
#define VISP_HAND2EYE_CALIBRATION_MESSAGE_COMPUTE_EFFECTOR_CAMERAREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace visp_hand2eye_calibration
{
template <class ContainerAllocator>
struct compute_effector_cameraRequest_
{
  typedef compute_effector_cameraRequest_<ContainerAllocator> Type;

  compute_effector_cameraRequest_()
    {
    }
  compute_effector_cameraRequest_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::visp_hand2eye_calibration::compute_effector_cameraRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::visp_hand2eye_calibration::compute_effector_cameraRequest_<ContainerAllocator> const> ConstPtr;

}; // struct compute_effector_cameraRequest_

typedef ::visp_hand2eye_calibration::compute_effector_cameraRequest_<std::allocator<void> > compute_effector_cameraRequest;

typedef boost::shared_ptr< ::visp_hand2eye_calibration::compute_effector_cameraRequest > compute_effector_cameraRequestPtr;
typedef boost::shared_ptr< ::visp_hand2eye_calibration::compute_effector_cameraRequest const> compute_effector_cameraRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::visp_hand2eye_calibration::compute_effector_cameraRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::visp_hand2eye_calibration::compute_effector_cameraRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace visp_hand2eye_calibration

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1339814/matlab/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg'], 'visp_hand2eye_calibration': ['/mathworks/home/pmurali/Documents/S/matlab_msg_gen_ros1/glnxa64/src/visp_hand2eye_calibration/msg'], 'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1339814/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg', '/mathworks/devel/sbs/31/pmurali.Brobot.j1339814/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::visp_hand2eye_calibration::compute_effector_cameraRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::visp_hand2eye_calibration::compute_effector_cameraRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::visp_hand2eye_calibration::compute_effector_cameraRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::visp_hand2eye_calibration::compute_effector_cameraRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::visp_hand2eye_calibration::compute_effector_cameraRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::visp_hand2eye_calibration::compute_effector_cameraRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::visp_hand2eye_calibration::compute_effector_cameraRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::visp_hand2eye_calibration::compute_effector_cameraRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::visp_hand2eye_calibration::compute_effector_cameraRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "visp_hand2eye_calibration/compute_effector_cameraRequest";
  }

  static const char* value(const ::visp_hand2eye_calibration::compute_effector_cameraRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::visp_hand2eye_calibration::compute_effector_cameraRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"\n"
"\n"
"\n"
;
  }

  static const char* value(const ::visp_hand2eye_calibration::compute_effector_cameraRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::visp_hand2eye_calibration::compute_effector_cameraRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct compute_effector_cameraRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::visp_hand2eye_calibration::compute_effector_cameraRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::visp_hand2eye_calibration::compute_effector_cameraRequest_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // VISP_HAND2EYE_CALIBRATION_MESSAGE_COMPUTE_EFFECTOR_CAMERAREQUEST_H
