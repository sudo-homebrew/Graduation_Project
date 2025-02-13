// Generated by gencpp from file ethercat_trigger_controllers/MultiWaveformTransition.msg
// DO NOT EDIT!


#ifndef ETHERCAT_TRIGGER_CONTROLLERS_MESSAGE_MULTIWAVEFORMTRANSITION_H
#define ETHERCAT_TRIGGER_CONTROLLERS_MESSAGE_MULTIWAVEFORMTRANSITION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ethercat_trigger_controllers
{
template <class ContainerAllocator>
struct MultiWaveformTransition_
{
  typedef MultiWaveformTransition_<ContainerAllocator> Type;

  MultiWaveformTransition_()
    : time(0.0)
    , value(0)
    , topic()  {
    }
  MultiWaveformTransition_(const ContainerAllocator& _alloc)
    : time(0.0)
    , value(0)
    , topic(_alloc)  {
  (void)_alloc;
    }



   typedef double _time_type;
  _time_type time;

   typedef uint32_t _value_type;
  _value_type value;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _topic_type;
  _topic_type topic;





  typedef boost::shared_ptr< ::ethercat_trigger_controllers::MultiWaveformTransition_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ethercat_trigger_controllers::MultiWaveformTransition_<ContainerAllocator> const> ConstPtr;

}; // struct MultiWaveformTransition_

typedef ::ethercat_trigger_controllers::MultiWaveformTransition_<std::allocator<void> > MultiWaveformTransition;

typedef boost::shared_ptr< ::ethercat_trigger_controllers::MultiWaveformTransition > MultiWaveformTransitionPtr;
typedef boost::shared_ptr< ::ethercat_trigger_controllers::MultiWaveformTransition const> MultiWaveformTransitionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ethercat_trigger_controllers::MultiWaveformTransition_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ethercat_trigger_controllers::MultiWaveformTransition_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ethercat_trigger_controllers

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1329247.2/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'ethercat_trigger_controllers': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/ethercat_trigger_controllers/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ethercat_trigger_controllers::MultiWaveformTransition_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ethercat_trigger_controllers::MultiWaveformTransition_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ethercat_trigger_controllers::MultiWaveformTransition_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ethercat_trigger_controllers::MultiWaveformTransition_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ethercat_trigger_controllers::MultiWaveformTransition_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ethercat_trigger_controllers::MultiWaveformTransition_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ethercat_trigger_controllers::MultiWaveformTransition_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bdd47e5d1c3d77473af2df9833a0608a";
  }

  static const char* value(const ::ethercat_trigger_controllers::MultiWaveformTransition_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbdd47e5d1c3d7747ULL;
  static const uint64_t static_value2 = 0x3af2df9833a0608aULL;
};

template<class ContainerAllocator>
struct DataType< ::ethercat_trigger_controllers::MultiWaveformTransition_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ethercat_trigger_controllers/MultiWaveformTransition";
  }

  static const char* value(const ::ethercat_trigger_controllers::MultiWaveformTransition_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ethercat_trigger_controllers::MultiWaveformTransition_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Used to specify a transition in the SetMultiWaveform service.\n"
"\n"
"float64 time # Transition time after start of period.\n"
"uint32 value # Value of the digital output after the transition time.\n"
"string topic # Topic to publish the transition timestamp to, or empty string if the transition should not be published.\n"
;
  }

  static const char* value(const ::ethercat_trigger_controllers::MultiWaveformTransition_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ethercat_trigger_controllers::MultiWaveformTransition_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.time);
      stream.next(m.value);
      stream.next(m.topic);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MultiWaveformTransition_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ethercat_trigger_controllers::MultiWaveformTransition_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ethercat_trigger_controllers::MultiWaveformTransition_<ContainerAllocator>& v)
  {
    s << indent << "time: ";
    Printer<double>::stream(s, indent + "  ", v.time);
    s << indent << "value: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.value);
    s << indent << "topic: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.topic);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ETHERCAT_TRIGGER_CONTROLLERS_MESSAGE_MULTIWAVEFORMTRANSITION_H
