// Generated by gencpp from file ethercat_hardware/RawFTData.msg
// DO NOT EDIT!


#ifndef ETHERCAT_HARDWARE_MESSAGE_RAWFTDATA_H
#define ETHERCAT_HARDWARE_MESSAGE_RAWFTDATA_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <ethercat_hardware/RawFTDataSample.h>

namespace ethercat_hardware
{
template <class ContainerAllocator>
struct RawFTData_
{
  typedef RawFTData_<ContainerAllocator> Type;

  RawFTData_()
    : samples()
    , sample_count(0)
    , missed_samples(0)  {
    }
  RawFTData_(const ContainerAllocator& _alloc)
    : samples(_alloc)
    , sample_count(0)
    , missed_samples(0)  {
  (void)_alloc;
    }



   typedef std::vector< ::ethercat_hardware::RawFTDataSample_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::ethercat_hardware::RawFTDataSample_<ContainerAllocator> >::other >  _samples_type;
  _samples_type samples;

   typedef int64_t _sample_count_type;
  _sample_count_type sample_count;

   typedef int64_t _missed_samples_type;
  _missed_samples_type missed_samples;





  typedef boost::shared_ptr< ::ethercat_hardware::RawFTData_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ethercat_hardware::RawFTData_<ContainerAllocator> const> ConstPtr;

}; // struct RawFTData_

typedef ::ethercat_hardware::RawFTData_<std::allocator<void> > RawFTData;

typedef boost::shared_ptr< ::ethercat_hardware::RawFTData > RawFTDataPtr;
typedef boost::shared_ptr< ::ethercat_hardware::RawFTData const> RawFTDataConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ethercat_hardware::RawFTData_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ethercat_hardware::RawFTData_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ethercat_hardware

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1329247.2/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg', '/mathworks/devel/sbs/31/pmurali.Brobot.j1329247.2/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'ethercat_hardware': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/ethercat_hardware/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ethercat_hardware::RawFTData_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ethercat_hardware::RawFTData_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ethercat_hardware::RawFTData_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ethercat_hardware::RawFTData_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ethercat_hardware::RawFTData_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ethercat_hardware::RawFTData_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ethercat_hardware::RawFTData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "85f5ed45095367bfb8fb2e57954c0b89";
  }

  static const char* value(const ::ethercat_hardware::RawFTData_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x85f5ed45095367bfULL;
  static const uint64_t static_value2 = 0xb8fb2e57954c0b89ULL;
};

template<class ContainerAllocator>
struct DataType< ::ethercat_hardware::RawFTData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ethercat_hardware/RawFTData";
  }

  static const char* value(const ::ethercat_hardware::RawFTData_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ethercat_hardware::RawFTData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Raw Data from WG035 F/T input via WG006 (gripper MCB).\n"
"RawFTDataSample[] samples  # The realtime loop receives upto 4 new samples each 1hKhz cycle \n"
"int64 sample_count         # Counts number of samples\n"
"int64 missed_samples       # Counts number of samples that were missed\n"
"================================================================================\n"
"MSG: ethercat_hardware/RawFTDataSample\n"
"# One raw Data sample from WG035 F/T input via WG006 (gripper MCB).\n"
"uint64  sample_count\n"
"int16[] data\n"
"uint16  vhalf\n"
;
  }

  static const char* value(const ::ethercat_hardware::RawFTData_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ethercat_hardware::RawFTData_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.samples);
      stream.next(m.sample_count);
      stream.next(m.missed_samples);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RawFTData_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ethercat_hardware::RawFTData_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ethercat_hardware::RawFTData_<ContainerAllocator>& v)
  {
    s << indent << "samples[]" << std::endl;
    for (size_t i = 0; i < v.samples.size(); ++i)
    {
      s << indent << "  samples[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::ethercat_hardware::RawFTDataSample_<ContainerAllocator> >::stream(s, indent + "    ", v.samples[i]);
    }
    s << indent << "sample_count: ";
    Printer<int64_t>::stream(s, indent + "  ", v.sample_count);
    s << indent << "missed_samples: ";
    Printer<int64_t>::stream(s, indent + "  ", v.missed_samples);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ETHERCAT_HARDWARE_MESSAGE_RAWFTDATA_H
