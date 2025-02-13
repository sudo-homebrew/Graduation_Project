// Generated by gencpp from file bayesian_belief_networks/Observation.msg
// DO NOT EDIT!


#ifndef BAYESIAN_BELIEF_NETWORKS_MESSAGE_OBSERVATION_H
#define BAYESIAN_BELIEF_NETWORKS_MESSAGE_OBSERVATION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace bayesian_belief_networks
{
template <class ContainerAllocator>
struct Observation_
{
  typedef Observation_<ContainerAllocator> Type;

  Observation_()
    : node()
    , evidence()  {
    }
  Observation_(const ContainerAllocator& _alloc)
    : node(_alloc)
    , evidence(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _node_type;
  _node_type node;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _evidence_type;
  _evidence_type evidence;





  typedef boost::shared_ptr< ::bayesian_belief_networks::Observation_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::bayesian_belief_networks::Observation_<ContainerAllocator> const> ConstPtr;

}; // struct Observation_

typedef ::bayesian_belief_networks::Observation_<std::allocator<void> > Observation;

typedef boost::shared_ptr< ::bayesian_belief_networks::Observation > ObservationPtr;
typedef boost::shared_ptr< ::bayesian_belief_networks::Observation const> ObservationConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::bayesian_belief_networks::Observation_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::bayesian_belief_networks::Observation_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace bayesian_belief_networks

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1329247.2/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'bayesian_belief_networks': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/bayesian_belief_networks/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::bayesian_belief_networks::Observation_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bayesian_belief_networks::Observation_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bayesian_belief_networks::Observation_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bayesian_belief_networks::Observation_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bayesian_belief_networks::Observation_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bayesian_belief_networks::Observation_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::bayesian_belief_networks::Observation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "381015522e25503885bf04a57ab55e63";
  }

  static const char* value(const ::bayesian_belief_networks::Observation_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x381015522e255038ULL;
  static const uint64_t static_value2 = 0x85bf04a57ab55e63ULL;
};

template<class ContainerAllocator>
struct DataType< ::bayesian_belief_networks::Observation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bayesian_belief_networks/Observation";
  }

  static const char* value(const ::bayesian_belief_networks::Observation_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::bayesian_belief_networks::Observation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string node\n"
"string evidence\n"
;
  }

  static const char* value(const ::bayesian_belief_networks::Observation_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::bayesian_belief_networks::Observation_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.node);
      stream.next(m.evidence);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Observation_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::bayesian_belief_networks::Observation_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::bayesian_belief_networks::Observation_<ContainerAllocator>& v)
  {
    s << indent << "node: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.node);
    s << indent << "evidence: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.evidence);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BAYESIAN_BELIEF_NETWORKS_MESSAGE_OBSERVATION_H
