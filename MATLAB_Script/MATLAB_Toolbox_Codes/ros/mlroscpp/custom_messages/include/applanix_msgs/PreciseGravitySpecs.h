// Generated by gencpp from file applanix_msgs/PreciseGravitySpecs.msg
// DO NOT EDIT!


#ifndef APPLANIX_MSGS_MESSAGE_PRECISEGRAVITYSPECS_H
#define APPLANIX_MSGS_MESSAGE_PRECISEGRAVITYSPECS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace applanix_msgs
{
template <class ContainerAllocator>
struct PreciseGravitySpecs_
{
  typedef PreciseGravitySpecs_<ContainerAllocator> Type;

  PreciseGravitySpecs_()
    : transaction(0)
    , gravity_magnitude(0.0)
    , north_deflection(0.0)
    , east_deflection(0.0)
    , latitude_of_validity(0.0)
    , longitude_of_validity(0.0)
    , altitude_of_validity(0.0)  {
    }
  PreciseGravitySpecs_(const ContainerAllocator& _alloc)
    : transaction(0)
    , gravity_magnitude(0.0)
    , north_deflection(0.0)
    , east_deflection(0.0)
    , latitude_of_validity(0.0)
    , longitude_of_validity(0.0)
    , altitude_of_validity(0.0)  {
  (void)_alloc;
    }



   typedef uint16_t _transaction_type;
  _transaction_type transaction;

   typedef double _gravity_magnitude_type;
  _gravity_magnitude_type gravity_magnitude;

   typedef double _north_deflection_type;
  _north_deflection_type north_deflection;

   typedef double _east_deflection_type;
  _east_deflection_type east_deflection;

   typedef double _latitude_of_validity_type;
  _latitude_of_validity_type latitude_of_validity;

   typedef double _longitude_of_validity_type;
  _longitude_of_validity_type longitude_of_validity;

   typedef double _altitude_of_validity_type;
  _altitude_of_validity_type altitude_of_validity;





  typedef boost::shared_ptr< ::applanix_msgs::PreciseGravitySpecs_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::applanix_msgs::PreciseGravitySpecs_<ContainerAllocator> const> ConstPtr;

}; // struct PreciseGravitySpecs_

typedef ::applanix_msgs::PreciseGravitySpecs_<std::allocator<void> > PreciseGravitySpecs;

typedef boost::shared_ptr< ::applanix_msgs::PreciseGravitySpecs > PreciseGravitySpecsPtr;
typedef boost::shared_ptr< ::applanix_msgs::PreciseGravitySpecs const> PreciseGravitySpecsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::applanix_msgs::PreciseGravitySpecs_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::applanix_msgs::PreciseGravitySpecs_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::applanix_msgs::PreciseGravitySpecs_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::applanix_msgs::PreciseGravitySpecs_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::applanix_msgs::PreciseGravitySpecs_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::applanix_msgs::PreciseGravitySpecs_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::applanix_msgs::PreciseGravitySpecs_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::applanix_msgs::PreciseGravitySpecs_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::applanix_msgs::PreciseGravitySpecs_<ContainerAllocator> >
{
  static const char* value()
  {
    return "60a1e48489b4ff46feb6027343f9931a";
  }

  static const char* value(const ::applanix_msgs::PreciseGravitySpecs_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x60a1e48489b4ff46ULL;
  static const uint64_t static_value2 = 0xfeb6027343f9931aULL;
};

template<class ContainerAllocator>
struct DataType< ::applanix_msgs::PreciseGravitySpecs_<ContainerAllocator> >
{
  static const char* value()
  {
    return "applanix_msgs/PreciseGravitySpecs";
  }

  static const char* value(const ::applanix_msgs::PreciseGravitySpecs_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::applanix_msgs::PreciseGravitySpecs_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Msg 40\n"
"uint16 transaction\n"
"\n"
"float64 gravity_magnitude\n"
"\n"
"float64 north_deflection\n"
"float64 east_deflection\n"
"\n"
"float64 latitude_of_validity\n"
"float64 longitude_of_validity\n"
"float64 altitude_of_validity\n"
;
  }

  static const char* value(const ::applanix_msgs::PreciseGravitySpecs_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::applanix_msgs::PreciseGravitySpecs_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.transaction);
      stream.next(m.gravity_magnitude);
      stream.next(m.north_deflection);
      stream.next(m.east_deflection);
      stream.next(m.latitude_of_validity);
      stream.next(m.longitude_of_validity);
      stream.next(m.altitude_of_validity);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PreciseGravitySpecs_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::applanix_msgs::PreciseGravitySpecs_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::applanix_msgs::PreciseGravitySpecs_<ContainerAllocator>& v)
  {
    s << indent << "transaction: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.transaction);
    s << indent << "gravity_magnitude: ";
    Printer<double>::stream(s, indent + "  ", v.gravity_magnitude);
    s << indent << "north_deflection: ";
    Printer<double>::stream(s, indent + "  ", v.north_deflection);
    s << indent << "east_deflection: ";
    Printer<double>::stream(s, indent + "  ", v.east_deflection);
    s << indent << "latitude_of_validity: ";
    Printer<double>::stream(s, indent + "  ", v.latitude_of_validity);
    s << indent << "longitude_of_validity: ";
    Printer<double>::stream(s, indent + "  ", v.longitude_of_validity);
    s << indent << "altitude_of_validity: ";
    Printer<double>::stream(s, indent + "  ", v.altitude_of_validity);
  }
};

} // namespace message_operations
} // namespace ros

#endif // APPLANIX_MSGS_MESSAGE_PRECISEGRAVITYSPECS_H