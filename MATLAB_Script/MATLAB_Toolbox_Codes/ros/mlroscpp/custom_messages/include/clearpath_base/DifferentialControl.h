// Generated by gencpp from file clearpath_base/DifferentialControl.msg
// DO NOT EDIT!


#ifndef CLEARPATH_BASE_MESSAGE_DIFFERENTIALCONTROL_H
#define CLEARPATH_BASE_MESSAGE_DIFFERENTIALCONTROL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace clearpath_base
{
template <class ContainerAllocator>
struct DifferentialControl_
{
  typedef DifferentialControl_<ContainerAllocator> Type;

  DifferentialControl_()
    : header()
    , l_p(0.0)
    , l_i(0.0)
    , l_d(0.0)
    , l_ffwd(0.0)
    , l_stic(0.0)
    , l_sat(0.0)
    , r_p(0.0)
    , r_i(0.0)
    , r_d(0.0)
    , r_ffwd(0.0)
    , r_stic(0.0)
    , r_sat(0.0)  {
    }
  DifferentialControl_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , l_p(0.0)
    , l_i(0.0)
    , l_d(0.0)
    , l_ffwd(0.0)
    , l_stic(0.0)
    , l_sat(0.0)
    , r_p(0.0)
    , r_i(0.0)
    , r_d(0.0)
    , r_ffwd(0.0)
    , r_stic(0.0)
    , r_sat(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _l_p_type;
  _l_p_type l_p;

   typedef double _l_i_type;
  _l_i_type l_i;

   typedef double _l_d_type;
  _l_d_type l_d;

   typedef double _l_ffwd_type;
  _l_ffwd_type l_ffwd;

   typedef double _l_stic_type;
  _l_stic_type l_stic;

   typedef double _l_sat_type;
  _l_sat_type l_sat;

   typedef double _r_p_type;
  _r_p_type r_p;

   typedef double _r_i_type;
  _r_i_type r_i;

   typedef double _r_d_type;
  _r_d_type r_d;

   typedef double _r_ffwd_type;
  _r_ffwd_type r_ffwd;

   typedef double _r_stic_type;
  _r_stic_type r_stic;

   typedef double _r_sat_type;
  _r_sat_type r_sat;





  typedef boost::shared_ptr< ::clearpath_base::DifferentialControl_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::clearpath_base::DifferentialControl_<ContainerAllocator> const> ConstPtr;

}; // struct DifferentialControl_

typedef ::clearpath_base::DifferentialControl_<std::allocator<void> > DifferentialControl;

typedef boost::shared_ptr< ::clearpath_base::DifferentialControl > DifferentialControlPtr;
typedef boost::shared_ptr< ::clearpath_base::DifferentialControl const> DifferentialControlConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::clearpath_base::DifferentialControl_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::clearpath_base::DifferentialControl_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::clearpath_base::DifferentialControl_<ContainerAllocator1> & lhs, const ::clearpath_base::DifferentialControl_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.l_p == rhs.l_p &&
    lhs.l_i == rhs.l_i &&
    lhs.l_d == rhs.l_d &&
    lhs.l_ffwd == rhs.l_ffwd &&
    lhs.l_stic == rhs.l_stic &&
    lhs.l_sat == rhs.l_sat &&
    lhs.r_p == rhs.r_p &&
    lhs.r_i == rhs.r_i &&
    lhs.r_d == rhs.r_d &&
    lhs.r_ffwd == rhs.r_ffwd &&
    lhs.r_stic == rhs.r_stic &&
    lhs.r_sat == rhs.r_sat;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::clearpath_base::DifferentialControl_<ContainerAllocator1> & lhs, const ::clearpath_base::DifferentialControl_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace clearpath_base

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::clearpath_base::DifferentialControl_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::clearpath_base::DifferentialControl_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::clearpath_base::DifferentialControl_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::clearpath_base::DifferentialControl_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::clearpath_base::DifferentialControl_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::clearpath_base::DifferentialControl_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::clearpath_base::DifferentialControl_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ae0672163e13fc0bb6491960c53a3259";
  }

  static const char* value(const ::clearpath_base::DifferentialControl_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xae0672163e13fc0bULL;
  static const uint64_t static_value2 = 0xb6491960c53a3259ULL;
};

template<class ContainerAllocator>
struct DataType< ::clearpath_base::DifferentialControl_<ContainerAllocator> >
{
  static const char* value()
  {
    return "clearpath_base/DifferentialControl";
  }

  static const char* value(const ::clearpath_base::DifferentialControl_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::clearpath_base::DifferentialControl_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"float64 l_p\n"
"float64 l_i\n"
"float64 l_d\n"
"float64 l_ffwd\n"
"float64 l_stic\n"
"float64 l_sat\n"
"float64 r_p\n"
"float64 r_i\n"
"float64 r_d\n"
"float64 r_ffwd\n"
"float64 r_stic\n"
"float64 r_sat\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::clearpath_base::DifferentialControl_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::clearpath_base::DifferentialControl_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.l_p);
      stream.next(m.l_i);
      stream.next(m.l_d);
      stream.next(m.l_ffwd);
      stream.next(m.l_stic);
      stream.next(m.l_sat);
      stream.next(m.r_p);
      stream.next(m.r_i);
      stream.next(m.r_d);
      stream.next(m.r_ffwd);
      stream.next(m.r_stic);
      stream.next(m.r_sat);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DifferentialControl_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::clearpath_base::DifferentialControl_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::clearpath_base::DifferentialControl_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "l_p: ";
    Printer<double>::stream(s, indent + "  ", v.l_p);
    s << indent << "l_i: ";
    Printer<double>::stream(s, indent + "  ", v.l_i);
    s << indent << "l_d: ";
    Printer<double>::stream(s, indent + "  ", v.l_d);
    s << indent << "l_ffwd: ";
    Printer<double>::stream(s, indent + "  ", v.l_ffwd);
    s << indent << "l_stic: ";
    Printer<double>::stream(s, indent + "  ", v.l_stic);
    s << indent << "l_sat: ";
    Printer<double>::stream(s, indent + "  ", v.l_sat);
    s << indent << "r_p: ";
    Printer<double>::stream(s, indent + "  ", v.r_p);
    s << indent << "r_i: ";
    Printer<double>::stream(s, indent + "  ", v.r_i);
    s << indent << "r_d: ";
    Printer<double>::stream(s, indent + "  ", v.r_d);
    s << indent << "r_ffwd: ";
    Printer<double>::stream(s, indent + "  ", v.r_ffwd);
    s << indent << "r_stic: ";
    Printer<double>::stream(s, indent + "  ", v.r_stic);
    s << indent << "r_sat: ";
    Printer<double>::stream(s, indent + "  ", v.r_sat);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CLEARPATH_BASE_MESSAGE_DIFFERENTIALCONTROL_H
