// Generated by gencpp from file applanix_msgs/NMEAMessageSelect.msg
// DO NOT EDIT!


#ifndef APPLANIX_MSGS_MESSAGE_NMEAMESSAGESELECT_H
#define APPLANIX_MSGS_MESSAGE_NMEAMESSAGESELECT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <applanix_msgs/COMPortMessages.h>

namespace applanix_msgs
{
template <class ContainerAllocator>
struct NMEAMessageSelect_
{
  typedef NMEAMessageSelect_<ContainerAllocator> Type;

  NMEAMessageSelect_()
    : transaction(0)
    , reserved()
    , talker(0)
    , ports_count(0)
    , ports()  {
      reserved.assign(0);
  }
  NMEAMessageSelect_(const ContainerAllocator& _alloc)
    : transaction(0)
    , reserved()
    , talker(0)
    , ports_count(0)
    , ports(_alloc)  {
  (void)_alloc;
      reserved.assign(0);
  }



   typedef uint16_t _transaction_type;
  _transaction_type transaction;

   typedef boost::array<uint8_t, 9>  _reserved_type;
  _reserved_type reserved;

   typedef uint8_t _talker_type;
  _talker_type talker;

   typedef uint8_t _ports_count_type;
  _ports_count_type ports_count;

   typedef std::vector< ::applanix_msgs::COMPortMessages_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::applanix_msgs::COMPortMessages_<ContainerAllocator> >::other >  _ports_type;
  _ports_type ports;



  enum {
    TALKER_IN = 0u,
    TALKER_GP = 1u,
  };


  typedef boost::shared_ptr< ::applanix_msgs::NMEAMessageSelect_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::applanix_msgs::NMEAMessageSelect_<ContainerAllocator> const> ConstPtr;

}; // struct NMEAMessageSelect_

typedef ::applanix_msgs::NMEAMessageSelect_<std::allocator<void> > NMEAMessageSelect;

typedef boost::shared_ptr< ::applanix_msgs::NMEAMessageSelect > NMEAMessageSelectPtr;
typedef boost::shared_ptr< ::applanix_msgs::NMEAMessageSelect const> NMEAMessageSelectConstPtr;

// constants requiring out of line definition

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::applanix_msgs::NMEAMessageSelect_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::applanix_msgs::NMEAMessageSelect_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace applanix_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/geometry_msgs/msg'], 'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1329247.2/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'applanix_msgs': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/applanix_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::applanix_msgs::NMEAMessageSelect_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::applanix_msgs::NMEAMessageSelect_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::applanix_msgs::NMEAMessageSelect_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::applanix_msgs::NMEAMessageSelect_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::applanix_msgs::NMEAMessageSelect_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::applanix_msgs::NMEAMessageSelect_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::applanix_msgs::NMEAMessageSelect_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d988d97031c9db73713ce27fde744cff";
  }

  static const char* value(const ::applanix_msgs::NMEAMessageSelect_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd988d97031c9db73ULL;
  static const uint64_t static_value2 = 0x713ce27fde744cffULL;
};

template<class ContainerAllocator>
struct DataType< ::applanix_msgs::NMEAMessageSelect_<ContainerAllocator> >
{
  static const char* value()
  {
    return "applanix_msgs/NMEAMessageSelect";
  }

  static const char* value(const ::applanix_msgs::NMEAMessageSelect_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::applanix_msgs::NMEAMessageSelect_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Msg 35\n"
"uint16 transaction\n"
"\n"
"uint8[9] reserved\n"
"\n"
"uint8 TALKER_IN=0\n"
"uint8 TALKER_GP=1\n"
"uint8 talker\n"
"\n"
"uint8 ports_count\n"
"COMPortMessages[] ports\n"
"\n"
"================================================================================\n"
"MSG: applanix_msgs/COMPortMessages\n"
"uint8 port_num\n"
"\n"
"uint32 MESSAGES_NMEA_GST=1\n"
"uint32 MESSAGES_NMEA_GGA=2\n"
"uint32 MESSAGES_NMEA_HDT=4\n"
"uint32 MESSAGES_NMEA_ZDA=8\n"
"uint32 MESSAGES_NMEA_EVT1=16\n"
"uint32 MESSAGES_NMEA_EVT2=32\n"
"uint32 MESSAGES_NMEA_VTG=64\n"
"uint32 MESSAGES_NMEA_PASHR=128\n"
"uint32 MESSAGES_NMEA_GGA2=8192\n"
"uint32 MESSAGES_NMEA_PPS=16384\n"
"uint32 MESSAGES_NMEA_GGK=32768\n"
"uint32 MESSAGES_NMEA_RMC=65536\n"
"uint32 MESSAGES_BIN_GIMBAL_LOOP=1\n"
"uint32 MESSAGES_BIN_RDR1=2\n"
"uint32 MESSAGES_BIN_PAST2=4\n"
"uint32 MESSAGES_BIN_PPS=65536\n"
"uint32 MESSAGES_BIN_TM1B=131072\n"
"uint32 messages\n"
"\n"
"uint8 update_rate\n"
;
  }

  static const char* value(const ::applanix_msgs::NMEAMessageSelect_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::applanix_msgs::NMEAMessageSelect_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.transaction);
      stream.next(m.reserved);
      stream.next(m.talker);
      stream.next(m.ports_count);
      stream.next(m.ports);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct NMEAMessageSelect_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::applanix_msgs::NMEAMessageSelect_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::applanix_msgs::NMEAMessageSelect_<ContainerAllocator>& v)
  {
    s << indent << "transaction: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.transaction);
    s << indent << "reserved[]" << std::endl;
    for (size_t i = 0; i < v.reserved.size(); ++i)
    {
      s << indent << "  reserved[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.reserved[i]);
    }
    s << indent << "talker: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.talker);
    s << indent << "ports_count: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.ports_count);
    s << indent << "ports[]" << std::endl;
    for (size_t i = 0; i < v.ports.size(); ++i)
    {
      s << indent << "  ports[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::applanix_msgs::COMPortMessages_<ContainerAllocator> >::stream(s, indent + "    ", v.ports[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // APPLANIX_MSGS_MESSAGE_NMEAMESSAGESELECT_H