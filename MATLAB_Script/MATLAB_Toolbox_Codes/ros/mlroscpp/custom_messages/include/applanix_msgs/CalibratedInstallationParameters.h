// Generated by gencpp from file applanix_msgs/CalibratedInstallationParameters.msg
// DO NOT EDIT!


#ifndef APPLANIX_MSGS_MESSAGE_CALIBRATEDINSTALLATIONPARAMETERS_H
#define APPLANIX_MSGS_MESSAGE_CALIBRATEDINSTALLATIONPARAMETERS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <applanix_msgs/TimeDistance.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point32.h>

namespace applanix_msgs
{
template <class ContainerAllocator>
struct CalibratedInstallationParameters_
{
  typedef CalibratedInstallationParameters_<ContainerAllocator> Type;

  CalibratedInstallationParameters_()
    : td()
    , status(0)
    , primary_gnss_lever_arm()
    , primary_gnss_lever_fom(0)
    , aux_1_gnss_lever_arm()
    , aux_1_gnss_lever_fom(0)
    , aux_2_gnss_lever_arm()
    , aux_2_gnss_lever_fom(0)
    , dmi_lever_arm()
    , dmi_lever_fom(0)
    , dmi_scale_factor(0.0)
    , dmi_scale_factor_fom(0)  {
    }
  CalibratedInstallationParameters_(const ContainerAllocator& _alloc)
    : td(_alloc)
    , status(0)
    , primary_gnss_lever_arm(_alloc)
    , primary_gnss_lever_fom(0)
    , aux_1_gnss_lever_arm(_alloc)
    , aux_1_gnss_lever_fom(0)
    , aux_2_gnss_lever_arm(_alloc)
    , aux_2_gnss_lever_fom(0)
    , dmi_lever_arm(_alloc)
    , dmi_lever_fom(0)
    , dmi_scale_factor(0.0)
    , dmi_scale_factor_fom(0)  {
  (void)_alloc;
    }



   typedef  ::applanix_msgs::TimeDistance_<ContainerAllocator>  _td_type;
  _td_type td;

   typedef uint16_t _status_type;
  _status_type status;

   typedef  ::geometry_msgs::Point32_<ContainerAllocator>  _primary_gnss_lever_arm_type;
  _primary_gnss_lever_arm_type primary_gnss_lever_arm;

   typedef uint16_t _primary_gnss_lever_fom_type;
  _primary_gnss_lever_fom_type primary_gnss_lever_fom;

   typedef  ::geometry_msgs::Point32_<ContainerAllocator>  _aux_1_gnss_lever_arm_type;
  _aux_1_gnss_lever_arm_type aux_1_gnss_lever_arm;

   typedef uint16_t _aux_1_gnss_lever_fom_type;
  _aux_1_gnss_lever_fom_type aux_1_gnss_lever_fom;

   typedef  ::geometry_msgs::Point32_<ContainerAllocator>  _aux_2_gnss_lever_arm_type;
  _aux_2_gnss_lever_arm_type aux_2_gnss_lever_arm;

   typedef uint16_t _aux_2_gnss_lever_fom_type;
  _aux_2_gnss_lever_fom_type aux_2_gnss_lever_fom;

   typedef  ::geometry_msgs::Point32_<ContainerAllocator>  _dmi_lever_arm_type;
  _dmi_lever_arm_type dmi_lever_arm;

   typedef uint16_t _dmi_lever_fom_type;
  _dmi_lever_fom_type dmi_lever_fom;

   typedef float _dmi_scale_factor_type;
  _dmi_scale_factor_type dmi_scale_factor;

   typedef uint16_t _dmi_scale_factor_fom_type;
  _dmi_scale_factor_fom_type dmi_scale_factor_fom;



  enum {
    STATUS_PRIMARY_GNSS_LEVER_ARM_CALIBRATING = 1u,
    STATUS_AUX_1_GNSS_LEVER_ARM_CALIBRATING = 2u,
    STATUS_AUX_2_GNSS_LEVER_ARM_CALIBRATING = 4u,
    STATUS_DMI_LEVER_ARM_CALIBRATING = 8u,
    STATUS_DMI_SCALE_FACTOR_CALIBRATING = 16u,
    STATUS_POSITION_FIX_LEVER_ARM_CALIBRATING = 64u,
    STATUS_PRIMARY_GNSS_LEVER_ARM_CALIBRATED = 256u,
    STATUS_AUX_1_GNSS_LEVER_ARM_CALIBRATED = 512u,
    STATUS_AUX_2_GNSS_LEVER_ARM_CALIBRATED = 1024u,
    STATUS_DMI_LEVER_ARM_CALIBRATED = 2048u,
    STATUS_DMI_SCALE_FACTOR_CALIBRATED = 4096u,
    STATUS_POSITION_FIX_LEVER_ARM_CALIBRATED = 16384u,
  };


  typedef boost::shared_ptr< ::applanix_msgs::CalibratedInstallationParameters_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::applanix_msgs::CalibratedInstallationParameters_<ContainerAllocator> const> ConstPtr;

}; // struct CalibratedInstallationParameters_

typedef ::applanix_msgs::CalibratedInstallationParameters_<std::allocator<void> > CalibratedInstallationParameters;

typedef boost::shared_ptr< ::applanix_msgs::CalibratedInstallationParameters > CalibratedInstallationParametersPtr;
typedef boost::shared_ptr< ::applanix_msgs::CalibratedInstallationParameters const> CalibratedInstallationParametersConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::applanix_msgs::CalibratedInstallationParameters_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::applanix_msgs::CalibratedInstallationParameters_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::applanix_msgs::CalibratedInstallationParameters_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::applanix_msgs::CalibratedInstallationParameters_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::applanix_msgs::CalibratedInstallationParameters_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::applanix_msgs::CalibratedInstallationParameters_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::applanix_msgs::CalibratedInstallationParameters_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::applanix_msgs::CalibratedInstallationParameters_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::applanix_msgs::CalibratedInstallationParameters_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1477542d348ed571610f2395143ec8d8";
  }

  static const char* value(const ::applanix_msgs::CalibratedInstallationParameters_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1477542d348ed571ULL;
  static const uint64_t static_value2 = 0x610f2395143ec8d8ULL;
};

template<class ContainerAllocator>
struct DataType< ::applanix_msgs::CalibratedInstallationParameters_<ContainerAllocator> >
{
  static const char* value()
  {
    return "applanix_msgs/CalibratedInstallationParameters";
  }

  static const char* value(const ::applanix_msgs::CalibratedInstallationParameters_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::applanix_msgs::CalibratedInstallationParameters_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Group 14\n"
"TimeDistance td\n"
"\n"
"# Bitfield\n"
"uint16 STATUS_PRIMARY_GNSS_LEVER_ARM_CALIBRATING=1\n"
"uint16 STATUS_AUX_1_GNSS_LEVER_ARM_CALIBRATING=2\n"
"uint16 STATUS_AUX_2_GNSS_LEVER_ARM_CALIBRATING=4\n"
"uint16 STATUS_DMI_LEVER_ARM_CALIBRATING=8\n"
"uint16 STATUS_DMI_SCALE_FACTOR_CALIBRATING=16\n"
"uint16 STATUS_POSITION_FIX_LEVER_ARM_CALIBRATING=64\n"
"uint16 STATUS_PRIMARY_GNSS_LEVER_ARM_CALIBRATED=256\n"
"uint16 STATUS_AUX_1_GNSS_LEVER_ARM_CALIBRATED=512\n"
"uint16 STATUS_AUX_2_GNSS_LEVER_ARM_CALIBRATED=1024\n"
"uint16 STATUS_DMI_LEVER_ARM_CALIBRATED=2048\n"
"uint16 STATUS_DMI_SCALE_FACTOR_CALIBRATED=4096\n"
"uint16 STATUS_POSITION_FIX_LEVER_ARM_CALIBRATED=16384\n"
"uint16 status\n"
"\n"
"geometry_msgs/Point32 primary_gnss_lever_arm\n"
"uint16 primary_gnss_lever_fom\n"
"\n"
"geometry_msgs/Point32 aux_1_gnss_lever_arm\n"
"uint16 aux_1_gnss_lever_fom\n"
"\n"
"geometry_msgs/Point32 aux_2_gnss_lever_arm\n"
"uint16 aux_2_gnss_lever_fom\n"
"\n"
"geometry_msgs/Point32 dmi_lever_arm\n"
"uint16 dmi_lever_fom\n"
"\n"
"float32 dmi_scale_factor\n"
"uint16 dmi_scale_factor_fom\n"
"\n"
"================================================================================\n"
"MSG: applanix_msgs/TimeDistance\n"
"float64 time1\n"
"float64 time2\n"
"float64 distance\n"
"uint8 time_types\n"
"uint8 distance_type\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point32\n"
"# This contains the position of a point in free space(with 32 bits of precision).\n"
"# It is recommeded to use Point wherever possible instead of Point32.  \n"
"# \n"
"# This recommendation is to promote interoperability.  \n"
"#\n"
"# This message is designed to take up less space when sending\n"
"# lots of points at once, as in the case of a PointCloud.  \n"
"\n"
"float32 x\n"
"float32 y\n"
"float32 z\n"
;
  }

  static const char* value(const ::applanix_msgs::CalibratedInstallationParameters_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::applanix_msgs::CalibratedInstallationParameters_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.td);
      stream.next(m.status);
      stream.next(m.primary_gnss_lever_arm);
      stream.next(m.primary_gnss_lever_fom);
      stream.next(m.aux_1_gnss_lever_arm);
      stream.next(m.aux_1_gnss_lever_fom);
      stream.next(m.aux_2_gnss_lever_arm);
      stream.next(m.aux_2_gnss_lever_fom);
      stream.next(m.dmi_lever_arm);
      stream.next(m.dmi_lever_fom);
      stream.next(m.dmi_scale_factor);
      stream.next(m.dmi_scale_factor_fom);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CalibratedInstallationParameters_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::applanix_msgs::CalibratedInstallationParameters_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::applanix_msgs::CalibratedInstallationParameters_<ContainerAllocator>& v)
  {
    s << indent << "td: ";
    s << std::endl;
    Printer< ::applanix_msgs::TimeDistance_<ContainerAllocator> >::stream(s, indent + "  ", v.td);
    s << indent << "status: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.status);
    s << indent << "primary_gnss_lever_arm: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point32_<ContainerAllocator> >::stream(s, indent + "  ", v.primary_gnss_lever_arm);
    s << indent << "primary_gnss_lever_fom: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.primary_gnss_lever_fom);
    s << indent << "aux_1_gnss_lever_arm: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point32_<ContainerAllocator> >::stream(s, indent + "  ", v.aux_1_gnss_lever_arm);
    s << indent << "aux_1_gnss_lever_fom: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.aux_1_gnss_lever_fom);
    s << indent << "aux_2_gnss_lever_arm: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point32_<ContainerAllocator> >::stream(s, indent + "  ", v.aux_2_gnss_lever_arm);
    s << indent << "aux_2_gnss_lever_fom: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.aux_2_gnss_lever_fom);
    s << indent << "dmi_lever_arm: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point32_<ContainerAllocator> >::stream(s, indent + "  ", v.dmi_lever_arm);
    s << indent << "dmi_lever_fom: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.dmi_lever_fom);
    s << indent << "dmi_scale_factor: ";
    Printer<float>::stream(s, indent + "  ", v.dmi_scale_factor);
    s << indent << "dmi_scale_factor_fom: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.dmi_scale_factor_fom);
  }
};

} // namespace message_operations
} // namespace ros

#endif // APPLANIX_MSGS_MESSAGE_CALIBRATEDINSTALLATIONPARAMETERS_H