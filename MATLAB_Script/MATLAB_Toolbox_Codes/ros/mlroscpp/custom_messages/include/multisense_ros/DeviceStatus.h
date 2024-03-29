// Generated by gencpp from file multisense_ros/DeviceStatus.msg
// DO NOT EDIT!


#ifndef MULTISENSE_ROS_MESSAGE_DEVICESTATUS_H
#define MULTISENSE_ROS_MESSAGE_DEVICESTATUS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace multisense_ros
{
template <class ContainerAllocator>
struct DeviceStatus_
{
  typedef DeviceStatus_<ContainerAllocator> Type;

  DeviceStatus_()
    : time()
    , uptime()
    , systemOk(false)
    , laserOk(false)
    , laserMotorOk(false)
    , camerasOk(false)
    , imuOk(false)
    , externalLedsOk(false)
    , processingPipelineOk(false)
    , powerSupplyTemp(0.0)
    , fpgaTemp(0.0)
    , leftImagerTemp(0.0)
    , rightImagerTemp(0.0)
    , inputVoltage(0.0)
    , inputCurrent(0.0)
    , fpgaPower(0.0)
    , logicPower(0.0)
    , imagerPower(0.0)  {
    }
  DeviceStatus_(const ContainerAllocator& _alloc)
    : time()
    , uptime()
    , systemOk(false)
    , laserOk(false)
    , laserMotorOk(false)
    , camerasOk(false)
    , imuOk(false)
    , externalLedsOk(false)
    , processingPipelineOk(false)
    , powerSupplyTemp(0.0)
    , fpgaTemp(0.0)
    , leftImagerTemp(0.0)
    , rightImagerTemp(0.0)
    , inputVoltage(0.0)
    , inputCurrent(0.0)
    , fpgaPower(0.0)
    , logicPower(0.0)
    , imagerPower(0.0)  {
  (void)_alloc;
    }



   typedef ros::Time _time_type;
  _time_type time;

   typedef ros::Time _uptime_type;
  _uptime_type uptime;

   typedef uint8_t _systemOk_type;
  _systemOk_type systemOk;

   typedef uint8_t _laserOk_type;
  _laserOk_type laserOk;

   typedef uint8_t _laserMotorOk_type;
  _laserMotorOk_type laserMotorOk;

   typedef uint8_t _camerasOk_type;
  _camerasOk_type camerasOk;

   typedef uint8_t _imuOk_type;
  _imuOk_type imuOk;

   typedef uint8_t _externalLedsOk_type;
  _externalLedsOk_type externalLedsOk;

   typedef uint8_t _processingPipelineOk_type;
  _processingPipelineOk_type processingPipelineOk;

   typedef float _powerSupplyTemp_type;
  _powerSupplyTemp_type powerSupplyTemp;

   typedef float _fpgaTemp_type;
  _fpgaTemp_type fpgaTemp;

   typedef float _leftImagerTemp_type;
  _leftImagerTemp_type leftImagerTemp;

   typedef float _rightImagerTemp_type;
  _rightImagerTemp_type rightImagerTemp;

   typedef float _inputVoltage_type;
  _inputVoltage_type inputVoltage;

   typedef float _inputCurrent_type;
  _inputCurrent_type inputCurrent;

   typedef float _fpgaPower_type;
  _fpgaPower_type fpgaPower;

   typedef float _logicPower_type;
  _logicPower_type logicPower;

   typedef float _imagerPower_type;
  _imagerPower_type imagerPower;





  typedef boost::shared_ptr< ::multisense_ros::DeviceStatus_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::multisense_ros::DeviceStatus_<ContainerAllocator> const> ConstPtr;

}; // struct DeviceStatus_

typedef ::multisense_ros::DeviceStatus_<std::allocator<void> > DeviceStatus;

typedef boost::shared_ptr< ::multisense_ros::DeviceStatus > DeviceStatusPtr;
typedef boost::shared_ptr< ::multisense_ros::DeviceStatus const> DeviceStatusConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::multisense_ros::DeviceStatus_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::multisense_ros::DeviceStatus_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::multisense_ros::DeviceStatus_<ContainerAllocator1> & lhs, const ::multisense_ros::DeviceStatus_<ContainerAllocator2> & rhs)
{
  return lhs.time == rhs.time &&
    lhs.uptime == rhs.uptime &&
    lhs.systemOk == rhs.systemOk &&
    lhs.laserOk == rhs.laserOk &&
    lhs.laserMotorOk == rhs.laserMotorOk &&
    lhs.camerasOk == rhs.camerasOk &&
    lhs.imuOk == rhs.imuOk &&
    lhs.externalLedsOk == rhs.externalLedsOk &&
    lhs.processingPipelineOk == rhs.processingPipelineOk &&
    lhs.powerSupplyTemp == rhs.powerSupplyTemp &&
    lhs.fpgaTemp == rhs.fpgaTemp &&
    lhs.leftImagerTemp == rhs.leftImagerTemp &&
    lhs.rightImagerTemp == rhs.rightImagerTemp &&
    lhs.inputVoltage == rhs.inputVoltage &&
    lhs.inputCurrent == rhs.inputCurrent &&
    lhs.fpgaPower == rhs.fpgaPower &&
    lhs.logicPower == rhs.logicPower &&
    lhs.imagerPower == rhs.imagerPower;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::multisense_ros::DeviceStatus_<ContainerAllocator1> & lhs, const ::multisense_ros::DeviceStatus_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace multisense_ros

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::multisense_ros::DeviceStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::multisense_ros::DeviceStatus_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::multisense_ros::DeviceStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::multisense_ros::DeviceStatus_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::multisense_ros::DeviceStatus_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::multisense_ros::DeviceStatus_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::multisense_ros::DeviceStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2114c900161a6607a8d4a04b3cecd16b";
  }

  static const char* value(const ::multisense_ros::DeviceStatus_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2114c900161a6607ULL;
  static const uint64_t static_value2 = 0xa8d4a04b3cecd16bULL;
};

template<class ContainerAllocator>
struct DataType< ::multisense_ros::DeviceStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "multisense_ros/DeviceStatus";
  }

  static const char* value(const ::multisense_ros::DeviceStatus_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::multisense_ros::DeviceStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "time time\n"
"time uptime\n"
"bool systemOk\n"
"bool laserOk\n"
"bool laserMotorOk\n"
"bool camerasOk\n"
"bool imuOk\n"
"bool externalLedsOk\n"
"bool processingPipelineOk\n"
"float32 powerSupplyTemp\n"
"float32 fpgaTemp\n"
"float32 leftImagerTemp\n"
"float32 rightImagerTemp\n"
"float32 inputVoltage\n"
"float32 inputCurrent\n"
"float32 fpgaPower\n"
"float32 logicPower\n"
"float32 imagerPower\n"
;
  }

  static const char* value(const ::multisense_ros::DeviceStatus_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::multisense_ros::DeviceStatus_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.time);
      stream.next(m.uptime);
      stream.next(m.systemOk);
      stream.next(m.laserOk);
      stream.next(m.laserMotorOk);
      stream.next(m.camerasOk);
      stream.next(m.imuOk);
      stream.next(m.externalLedsOk);
      stream.next(m.processingPipelineOk);
      stream.next(m.powerSupplyTemp);
      stream.next(m.fpgaTemp);
      stream.next(m.leftImagerTemp);
      stream.next(m.rightImagerTemp);
      stream.next(m.inputVoltage);
      stream.next(m.inputCurrent);
      stream.next(m.fpgaPower);
      stream.next(m.logicPower);
      stream.next(m.imagerPower);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DeviceStatus_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::multisense_ros::DeviceStatus_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::multisense_ros::DeviceStatus_<ContainerAllocator>& v)
  {
    s << indent << "time: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.time);
    s << indent << "uptime: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.uptime);
    s << indent << "systemOk: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.systemOk);
    s << indent << "laserOk: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.laserOk);
    s << indent << "laserMotorOk: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.laserMotorOk);
    s << indent << "camerasOk: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.camerasOk);
    s << indent << "imuOk: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.imuOk);
    s << indent << "externalLedsOk: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.externalLedsOk);
    s << indent << "processingPipelineOk: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.processingPipelineOk);
    s << indent << "powerSupplyTemp: ";
    Printer<float>::stream(s, indent + "  ", v.powerSupplyTemp);
    s << indent << "fpgaTemp: ";
    Printer<float>::stream(s, indent + "  ", v.fpgaTemp);
    s << indent << "leftImagerTemp: ";
    Printer<float>::stream(s, indent + "  ", v.leftImagerTemp);
    s << indent << "rightImagerTemp: ";
    Printer<float>::stream(s, indent + "  ", v.rightImagerTemp);
    s << indent << "inputVoltage: ";
    Printer<float>::stream(s, indent + "  ", v.inputVoltage);
    s << indent << "inputCurrent: ";
    Printer<float>::stream(s, indent + "  ", v.inputCurrent);
    s << indent << "fpgaPower: ";
    Printer<float>::stream(s, indent + "  ", v.fpgaPower);
    s << indent << "logicPower: ";
    Printer<float>::stream(s, indent + "  ", v.logicPower);
    s << indent << "imagerPower: ";
    Printer<float>::stream(s, indent + "  ", v.imagerPower);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MULTISENSE_ROS_MESSAGE_DEVICESTATUS_H
