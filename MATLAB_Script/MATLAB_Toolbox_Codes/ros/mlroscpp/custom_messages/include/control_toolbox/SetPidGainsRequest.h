// Generated by gencpp from file control_toolbox/SetPidGainsRequest.msg
// DO NOT EDIT!


#ifndef CONTROL_TOOLBOX_MESSAGE_SETPIDGAINSREQUEST_H
#define CONTROL_TOOLBOX_MESSAGE_SETPIDGAINSREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace control_toolbox
{
template <class ContainerAllocator>
struct SetPidGainsRequest_
{
  typedef SetPidGainsRequest_<ContainerAllocator> Type;

  SetPidGainsRequest_()
    : p(0.0)
    , i(0.0)
    , d(0.0)
    , i_clamp(0.0)
    , antiwindup(false)  {
    }
  SetPidGainsRequest_(const ContainerAllocator& _alloc)
    : p(0.0)
    , i(0.0)
    , d(0.0)
    , i_clamp(0.0)
    , antiwindup(false)  {
  (void)_alloc;
    }



   typedef double _p_type;
  _p_type p;

   typedef double _i_type;
  _i_type i;

   typedef double _d_type;
  _d_type d;

   typedef double _i_clamp_type;
  _i_clamp_type i_clamp;

   typedef uint8_t _antiwindup_type;
  _antiwindup_type antiwindup;





  typedef boost::shared_ptr< ::control_toolbox::SetPidGainsRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::control_toolbox::SetPidGainsRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetPidGainsRequest_

typedef ::control_toolbox::SetPidGainsRequest_<std::allocator<void> > SetPidGainsRequest;

typedef boost::shared_ptr< ::control_toolbox::SetPidGainsRequest > SetPidGainsRequestPtr;
typedef boost::shared_ptr< ::control_toolbox::SetPidGainsRequest const> SetPidGainsRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::control_toolbox::SetPidGainsRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::control_toolbox::SetPidGainsRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::control_toolbox::SetPidGainsRequest_<ContainerAllocator1> & lhs, const ::control_toolbox::SetPidGainsRequest_<ContainerAllocator2> & rhs)
{
  return lhs.p == rhs.p &&
    lhs.i == rhs.i &&
    lhs.d == rhs.d &&
    lhs.i_clamp == rhs.i_clamp &&
    lhs.antiwindup == rhs.antiwindup;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::control_toolbox::SetPidGainsRequest_<ContainerAllocator1> & lhs, const ::control_toolbox::SetPidGainsRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace control_toolbox

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::control_toolbox::SetPidGainsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::control_toolbox::SetPidGainsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::control_toolbox::SetPidGainsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::control_toolbox::SetPidGainsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::control_toolbox::SetPidGainsRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::control_toolbox::SetPidGainsRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::control_toolbox::SetPidGainsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4a43159879643e60937bf2893b633607";
  }

  static const char* value(const ::control_toolbox::SetPidGainsRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4a43159879643e60ULL;
  static const uint64_t static_value2 = 0x937bf2893b633607ULL;
};

template<class ContainerAllocator>
struct DataType< ::control_toolbox::SetPidGainsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "control_toolbox/SetPidGainsRequest";
  }

  static const char* value(const ::control_toolbox::SetPidGainsRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::control_toolbox::SetPidGainsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 p\n"
"float64 i\n"
"float64 d\n"
"float64 i_clamp\n"
"bool antiwindup\n"
;
  }

  static const char* value(const ::control_toolbox::SetPidGainsRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::control_toolbox::SetPidGainsRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.p);
      stream.next(m.i);
      stream.next(m.d);
      stream.next(m.i_clamp);
      stream.next(m.antiwindup);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetPidGainsRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::control_toolbox::SetPidGainsRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::control_toolbox::SetPidGainsRequest_<ContainerAllocator>& v)
  {
    s << indent << "p: ";
    Printer<double>::stream(s, indent + "  ", v.p);
    s << indent << "i: ";
    Printer<double>::stream(s, indent + "  ", v.i);
    s << indent << "d: ";
    Printer<double>::stream(s, indent + "  ", v.d);
    s << indent << "i_clamp: ";
    Printer<double>::stream(s, indent + "  ", v.i_clamp);
    s << indent << "antiwindup: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.antiwindup);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CONTROL_TOOLBOX_MESSAGE_SETPIDGAINSREQUEST_H
