// Generated by gencpp from file cob_relayboard/EmergencyStopState.msg
// DO NOT EDIT!


#ifndef COB_RELAYBOARD_MESSAGE_EMERGENCYSTOPSTATE_H
#define COB_RELAYBOARD_MESSAGE_EMERGENCYSTOPSTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace cob_relayboard
{
template <class ContainerAllocator>
struct EmergencyStopState_
{
  typedef EmergencyStopState_<ContainerAllocator> Type;

  EmergencyStopState_()
    : emergency_button_stop(false)
    , scanner_stop(false)
    , emergency_state(0)  {
    }
  EmergencyStopState_(const ContainerAllocator& _alloc)
    : emergency_button_stop(false)
    , scanner_stop(false)
    , emergency_state(0)  {
  (void)_alloc;
    }



   typedef uint8_t _emergency_button_stop_type;
  _emergency_button_stop_type emergency_button_stop;

   typedef uint8_t _scanner_stop_type;
  _scanner_stop_type scanner_stop;

   typedef int16_t _emergency_state_type;
  _emergency_state_type emergency_state;



  enum {
    EMFREE = 0,
    EMSTOP = 1,
    EMCONFIRMED = 2,
  };


  typedef boost::shared_ptr< ::cob_relayboard::EmergencyStopState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cob_relayboard::EmergencyStopState_<ContainerAllocator> const> ConstPtr;

}; // struct EmergencyStopState_

typedef ::cob_relayboard::EmergencyStopState_<std::allocator<void> > EmergencyStopState;

typedef boost::shared_ptr< ::cob_relayboard::EmergencyStopState > EmergencyStopStatePtr;
typedef boost::shared_ptr< ::cob_relayboard::EmergencyStopState const> EmergencyStopStateConstPtr;

// constants requiring out of line definition

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cob_relayboard::EmergencyStopState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cob_relayboard::EmergencyStopState_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace cob_relayboard

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'cob_relayboard': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/cob_relayboard/msg'], 'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1329247.2/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::cob_relayboard::EmergencyStopState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cob_relayboard::EmergencyStopState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cob_relayboard::EmergencyStopState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cob_relayboard::EmergencyStopState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cob_relayboard::EmergencyStopState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cob_relayboard::EmergencyStopState_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cob_relayboard::EmergencyStopState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d857d7312ffc16f75239036504e493e9";
  }

  static const char* value(const ::cob_relayboard::EmergencyStopState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd857d7312ffc16f7ULL;
  static const uint64_t static_value2 = 0x5239036504e493e9ULL;
};

template<class ContainerAllocator>
struct DataType< ::cob_relayboard::EmergencyStopState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cob_relayboard/EmergencyStopState";
  }

  static const char* value(const ::cob_relayboard::EmergencyStopState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cob_relayboard::EmergencyStopState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This message holds the emergency stop (EMStop) status of the robot. It detects wether an EMStop is caused by the safety laserscanner or the emergency stop buttons. Moreover, it gives signalizes wether the EMStop was confirmed (after Button press stop) and the system is free again.\n"
"\n"
"# Possible EMStop States\n"
"int16 EMFREE = 0 		# system operatign normal\n"
"int16 EMSTOP = 1 		# emergency stop is active (Button pressed; obstacle in safety field of scanner)\n"
"int16 EMCONFIRMED = 2 	# emergency stop was confirmed system is reinitializing and going back to normal\n"
"\n"
"bool emergency_button_stop	# true = emergency stop signal is issued by button pressed\n"
"bool scanner_stop			# true = emergency stop signal is issued by scanner\n"
"int16 emergency_state		# state (including confimation by key-switch), values see above\n"
"\n"
;
  }

  static const char* value(const ::cob_relayboard::EmergencyStopState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cob_relayboard::EmergencyStopState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.emergency_button_stop);
      stream.next(m.scanner_stop);
      stream.next(m.emergency_state);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct EmergencyStopState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cob_relayboard::EmergencyStopState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cob_relayboard::EmergencyStopState_<ContainerAllocator>& v)
  {
    s << indent << "emergency_button_stop: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.emergency_button_stop);
    s << indent << "scanner_stop: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.scanner_stop);
    s << indent << "emergency_state: ";
    Printer<int16_t>::stream(s, indent + "  ", v.emergency_state);
  }
};

} // namespace message_operations
} // namespace ros

#endif // COB_RELAYBOARD_MESSAGE_EMERGENCYSTOPSTATE_H