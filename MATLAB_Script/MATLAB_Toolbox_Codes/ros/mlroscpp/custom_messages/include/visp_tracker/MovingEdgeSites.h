// Generated by gencpp from file visp_tracker/MovingEdgeSites.msg
// DO NOT EDIT!


#ifndef VISP_TRACKER_MESSAGE_MOVINGEDGESITES_H
#define VISP_TRACKER_MESSAGE_MOVINGEDGESITES_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <visp_tracker/MovingEdgeSite.h>

namespace visp_tracker
{
template <class ContainerAllocator>
struct MovingEdgeSites_
{
  typedef MovingEdgeSites_<ContainerAllocator> Type;

  MovingEdgeSites_()
    : header()
    , moving_edge_sites()  {
    }
  MovingEdgeSites_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , moving_edge_sites(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::visp_tracker::MovingEdgeSite_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::visp_tracker::MovingEdgeSite_<ContainerAllocator> >::other >  _moving_edge_sites_type;
  _moving_edge_sites_type moving_edge_sites;





  typedef boost::shared_ptr< ::visp_tracker::MovingEdgeSites_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::visp_tracker::MovingEdgeSites_<ContainerAllocator> const> ConstPtr;

}; // struct MovingEdgeSites_

typedef ::visp_tracker::MovingEdgeSites_<std::allocator<void> > MovingEdgeSites;

typedef boost::shared_ptr< ::visp_tracker::MovingEdgeSites > MovingEdgeSitesPtr;
typedef boost::shared_ptr< ::visp_tracker::MovingEdgeSites const> MovingEdgeSitesConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::visp_tracker::MovingEdgeSites_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::visp_tracker::MovingEdgeSites_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::visp_tracker::MovingEdgeSites_<ContainerAllocator1> & lhs, const ::visp_tracker::MovingEdgeSites_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.moving_edge_sites == rhs.moving_edge_sites;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::visp_tracker::MovingEdgeSites_<ContainerAllocator1> & lhs, const ::visp_tracker::MovingEdgeSites_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace visp_tracker

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::visp_tracker::MovingEdgeSites_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::visp_tracker::MovingEdgeSites_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::visp_tracker::MovingEdgeSites_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::visp_tracker::MovingEdgeSites_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::visp_tracker::MovingEdgeSites_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::visp_tracker::MovingEdgeSites_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::visp_tracker::MovingEdgeSites_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5293e8601467590a0dabbb34da47310c";
  }

  static const char* value(const ::visp_tracker::MovingEdgeSites_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5293e8601467590aULL;
  static const uint64_t static_value2 = 0x0dabbb34da47310cULL;
};

template<class ContainerAllocator>
struct DataType< ::visp_tracker::MovingEdgeSites_<ContainerAllocator> >
{
  static const char* value()
  {
    return "visp_tracker/MovingEdgeSites";
  }

  static const char* value(const ::visp_tracker::MovingEdgeSites_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::visp_tracker::MovingEdgeSites_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Stamped list of moving edge positions.\n"
"#\n"
"# Moving edge positions associated with a particular timestamp.\n"
"# Used by the viewer to display moving edge positions and allow\n"
"# tracking debug.\n"
"\n"
"Header header                       # Header (required for synchronization).\n"
"MovingEdgeSite[] moving_edge_sites  # List of moving dge sites (i.e. positions).\n"
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
"\n"
"================================================================================\n"
"MSG: visp_tracker/MovingEdgeSite\n"
"# Moving edge position.\n"
"\n"
"float64 x      # X position in the image\n"
"float64 y      # Y position in the image\n"
"int32 suppress # Is the moving edge valid?\n"
"               # - 0:   yes\n"
"	       # - 1:   no, constrast check has failed.\n"
"	       # - 2:   no, threshold check has failed.\n"
"	       # - 3:   no, M-estimator check has failed.\n"
"	       # - >=4: no, undocumented reason.\n"
;
  }

  static const char* value(const ::visp_tracker::MovingEdgeSites_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::visp_tracker::MovingEdgeSites_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.moving_edge_sites);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MovingEdgeSites_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::visp_tracker::MovingEdgeSites_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::visp_tracker::MovingEdgeSites_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "moving_edge_sites[]" << std::endl;
    for (size_t i = 0; i < v.moving_edge_sites.size(); ++i)
    {
      s << indent << "  moving_edge_sites[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::visp_tracker::MovingEdgeSite_<ContainerAllocator> >::stream(s, indent + "    ", v.moving_edge_sites[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // VISP_TRACKER_MESSAGE_MOVINGEDGESITES_H
