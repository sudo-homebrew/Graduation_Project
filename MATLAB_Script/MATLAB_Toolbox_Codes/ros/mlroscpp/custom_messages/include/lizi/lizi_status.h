// Generated by gencpp from file lizi/lizi_status.msg
// DO NOT EDIT!


#ifndef LIZI_MESSAGE_LIZI_STATUS_H
#define LIZI_MESSAGE_LIZI_STATUS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace lizi
{
template <class ContainerAllocator>
struct lizi_status_
{
  typedef lizi_status_<ContainerAllocator> Type;

  lizi_status_()
    : faults(0)
    , battery_voltage(0.0)  {
    }
  lizi_status_(const ContainerAllocator& _alloc)
    : faults(0)
    , battery_voltage(0.0)  {
  (void)_alloc;
    }



   typedef uint8_t _faults_type;
  _faults_type faults;

   typedef float _battery_voltage_type;
  _battery_voltage_type battery_voltage;





  typedef boost::shared_ptr< ::lizi::lizi_status_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::lizi::lizi_status_<ContainerAllocator> const> ConstPtr;

}; // struct lizi_status_

typedef ::lizi::lizi_status_<std::allocator<void> > lizi_status;

typedef boost::shared_ptr< ::lizi::lizi_status > lizi_statusPtr;
typedef boost::shared_ptr< ::lizi::lizi_status const> lizi_statusConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::lizi::lizi_status_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::lizi::lizi_status_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace lizi

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'lizi': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/lizi/msg'], 'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1329247.2/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::lizi::lizi_status_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lizi::lizi_status_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lizi::lizi_status_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lizi::lizi_status_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lizi::lizi_status_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lizi::lizi_status_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::lizi::lizi_status_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f66cc2fe91fb70d2b82c88e7c03227df";
  }

  static const char* value(const ::lizi::lizi_status_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf66cc2fe91fb70d2ULL;
  static const uint64_t static_value2 = 0xb82c88e7c03227dfULL;
};

template<class ContainerAllocator>
struct DataType< ::lizi::lizi_status_<ContainerAllocator> >
{
  static const char* value()
  {
    return "lizi/lizi_status";
  }

  static const char* value(const ::lizi::lizi_status_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::lizi::lizi_status_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 faults\n"
"float32 battery_voltage\n"
;
  }

  static const char* value(const ::lizi::lizi_status_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::lizi::lizi_status_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.faults);
      stream.next(m.battery_voltage);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct lizi_status_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::lizi::lizi_status_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::lizi::lizi_status_<ContainerAllocator>& v)
  {
    s << indent << "faults: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.faults);
    s << indent << "battery_voltage: ";
    Printer<float>::stream(s, indent + "  ", v.battery_voltage);
  }
};

} // namespace message_operations
} // namespace ros

#endif // LIZI_MESSAGE_LIZI_STATUS_H