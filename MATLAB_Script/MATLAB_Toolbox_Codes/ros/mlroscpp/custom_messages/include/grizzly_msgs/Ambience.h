// Generated by gencpp from file grizzly_msgs/Ambience.msg
// DO NOT EDIT!


#ifndef GRIZZLY_MSGS_MESSAGE_AMBIENCE_H
#define GRIZZLY_MSGS_MESSAGE_AMBIENCE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace grizzly_msgs
{
template <class ContainerAllocator>
struct Ambience_
{
  typedef Ambience_<ContainerAllocator> Type;

  Ambience_()
    : body_lights()
    , beacon(0)
    , beep(0)  {
      body_lights.assign(0);
  }
  Ambience_(const ContainerAllocator& _alloc)
    : body_lights()
    , beacon(0)
    , beep(0)  {
  (void)_alloc;
      body_lights.assign(0);
  }



   typedef boost::array<uint8_t, 4>  _body_lights_type;
  _body_lights_type body_lights;

   typedef uint8_t _beacon_type;
  _beacon_type beacon;

   typedef uint8_t _beep_type;
  _beep_type beep;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(LIGHTS_FRONT_LEFT)
  #undef LIGHTS_FRONT_LEFT
#endif
#if defined(_WIN32) && defined(LIGHTS_FRONT_RIGHT)
  #undef LIGHTS_FRONT_RIGHT
#endif
#if defined(_WIN32) && defined(LIGHTS_REAR_LEFT)
  #undef LIGHTS_REAR_LEFT
#endif
#if defined(_WIN32) && defined(LIGHTS_REAR_RIGHT)
  #undef LIGHTS_REAR_RIGHT
#endif

  enum {
    LIGHTS_FRONT_LEFT = 0u,
    LIGHTS_FRONT_RIGHT = 1u,
    LIGHTS_REAR_LEFT = 2u,
    LIGHTS_REAR_RIGHT = 3u,
  };


  typedef boost::shared_ptr< ::grizzly_msgs::Ambience_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::grizzly_msgs::Ambience_<ContainerAllocator> const> ConstPtr;

}; // struct Ambience_

typedef ::grizzly_msgs::Ambience_<std::allocator<void> > Ambience;

typedef boost::shared_ptr< ::grizzly_msgs::Ambience > AmbiencePtr;
typedef boost::shared_ptr< ::grizzly_msgs::Ambience const> AmbienceConstPtr;

// constants requiring out of line definition

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::grizzly_msgs::Ambience_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::grizzly_msgs::Ambience_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::grizzly_msgs::Ambience_<ContainerAllocator1> & lhs, const ::grizzly_msgs::Ambience_<ContainerAllocator2> & rhs)
{
  return lhs.body_lights == rhs.body_lights &&
    lhs.beacon == rhs.beacon &&
    lhs.beep == rhs.beep;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::grizzly_msgs::Ambience_<ContainerAllocator1> & lhs, const ::grizzly_msgs::Ambience_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace grizzly_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::grizzly_msgs::Ambience_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::grizzly_msgs::Ambience_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::grizzly_msgs::Ambience_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::grizzly_msgs::Ambience_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::grizzly_msgs::Ambience_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::grizzly_msgs::Ambience_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::grizzly_msgs::Ambience_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8271ef649dd1720612cc15f8990fdf6f";
  }

  static const char* value(const ::grizzly_msgs::Ambience_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8271ef649dd17206ULL;
  static const uint64_t static_value2 = 0x12cc15f8990fdf6fULL;
};

template<class ContainerAllocator>
struct DataType< ::grizzly_msgs::Ambience_<ContainerAllocator> >
{
  static const char* value()
  {
    return "grizzly_msgs/Ambience";
  }

  static const char* value(const ::grizzly_msgs::Ambience_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::grizzly_msgs::Ambience_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Represents a command for the four corner body lights, indicators on the rear\n"
"# and beeper/beacon combination on the arch.\n"
"uint8 LIGHTS_FRONT_LEFT=0   # Only white\n"
"uint8 LIGHTS_FRONT_RIGHT=1  # Only white\n"
"uint8 LIGHTS_REAR_LEFT=2    # Only red\n"
"uint8 LIGHTS_REAR_RIGHT=3   # Only red\n"
"\n"
"uint8[4] body_lights    # Body lights on the Grizzly\n"
"uint8 beacon            # Overhead beacon light\n"
"uint8 beep              # Noisemaker sound\n"
;
  }

  static const char* value(const ::grizzly_msgs::Ambience_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::grizzly_msgs::Ambience_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.body_lights);
      stream.next(m.beacon);
      stream.next(m.beep);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Ambience_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::grizzly_msgs::Ambience_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::grizzly_msgs::Ambience_<ContainerAllocator>& v)
  {
    s << indent << "body_lights[]" << std::endl;
    for (size_t i = 0; i < v.body_lights.size(); ++i)
    {
      s << indent << "  body_lights[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.body_lights[i]);
    }
    s << indent << "beacon: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.beacon);
    s << indent << "beep: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.beep);
  }
};

} // namespace message_operations
} // namespace ros

#endif // GRIZZLY_MSGS_MESSAGE_AMBIENCE_H