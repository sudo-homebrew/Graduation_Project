// Generated by gencpp from file visp_tracker/MovingEdgeSettings.msg
// DO NOT EDIT!


#ifndef VISP_TRACKER_MESSAGE_MOVINGEDGESETTINGS_H
#define VISP_TRACKER_MESSAGE_MOVINGEDGESETTINGS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace visp_tracker
{
template <class ContainerAllocator>
struct MovingEdgeSettings_
{
  typedef MovingEdgeSettings_<ContainerAllocator> Type;

  MovingEdgeSettings_()
    : mask_size(0)
    , range(0)
    , threshold(0.0)
    , mu1(0.0)
    , mu2(0.0)
    , sample_step(0)
    , strip(0)
    , first_threshold(0.0)  {
    }
  MovingEdgeSettings_(const ContainerAllocator& _alloc)
    : mask_size(0)
    , range(0)
    , threshold(0.0)
    , mu1(0.0)
    , mu2(0.0)
    , sample_step(0)
    , strip(0)
    , first_threshold(0.0)  {
  (void)_alloc;
    }



   typedef int64_t _mask_size_type;
  _mask_size_type mask_size;

   typedef int64_t _range_type;
  _range_type range;

   typedef double _threshold_type;
  _threshold_type threshold;

   typedef double _mu1_type;
  _mu1_type mu1;

   typedef double _mu2_type;
  _mu2_type mu2;

   typedef int64_t _sample_step_type;
  _sample_step_type sample_step;

   typedef int64_t _strip_type;
  _strip_type strip;

   typedef double _first_threshold_type;
  _first_threshold_type first_threshold;





  typedef boost::shared_ptr< ::visp_tracker::MovingEdgeSettings_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::visp_tracker::MovingEdgeSettings_<ContainerAllocator> const> ConstPtr;

}; // struct MovingEdgeSettings_

typedef ::visp_tracker::MovingEdgeSettings_<std::allocator<void> > MovingEdgeSettings;

typedef boost::shared_ptr< ::visp_tracker::MovingEdgeSettings > MovingEdgeSettingsPtr;
typedef boost::shared_ptr< ::visp_tracker::MovingEdgeSettings const> MovingEdgeSettingsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::visp_tracker::MovingEdgeSettings_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::visp_tracker::MovingEdgeSettings_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::visp_tracker::MovingEdgeSettings_<ContainerAllocator1> & lhs, const ::visp_tracker::MovingEdgeSettings_<ContainerAllocator2> & rhs)
{
  return lhs.mask_size == rhs.mask_size &&
    lhs.range == rhs.range &&
    lhs.threshold == rhs.threshold &&
    lhs.mu1 == rhs.mu1 &&
    lhs.mu2 == rhs.mu2 &&
    lhs.sample_step == rhs.sample_step &&
    lhs.strip == rhs.strip &&
    lhs.first_threshold == rhs.first_threshold;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::visp_tracker::MovingEdgeSettings_<ContainerAllocator1> & lhs, const ::visp_tracker::MovingEdgeSettings_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace visp_tracker

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::visp_tracker::MovingEdgeSettings_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::visp_tracker::MovingEdgeSettings_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::visp_tracker::MovingEdgeSettings_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::visp_tracker::MovingEdgeSettings_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::visp_tracker::MovingEdgeSettings_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::visp_tracker::MovingEdgeSettings_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::visp_tracker::MovingEdgeSettings_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4e8f98b7dec6ffa099529044b3472486";
  }

  static const char* value(const ::visp_tracker::MovingEdgeSettings_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4e8f98b7dec6ffa0ULL;
  static const uint64_t static_value2 = 0x99529044b3472486ULL;
};

template<class ContainerAllocator>
struct DataType< ::visp_tracker::MovingEdgeSettings_<ContainerAllocator> >
{
  static const char* value()
  {
    return "visp_tracker/MovingEdgeSettings";
  }

  static const char* value(const ::visp_tracker::MovingEdgeSettings_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::visp_tracker::MovingEdgeSettings_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This message contains tracking parameters.\n"
"#\n"
"# These parameters determine how precise, how fast and how\n"
"# reliable will be the tracking.\n"
"#\n"
"# It should be tuned carefully and can be changed dynamically.\n"
"#\n"
"# For more details, see the ViSP documentation:\n"
"# http://www.irisa.fr/lagadic/visp/publication.html\n"
"\n"
"\n"
"# Moving edge parameters.\n"
"\n"
"int64 mask_size    # Mask size (in pixel) used to compute the image gradient\n"
"                   # and determine the object contour.\n"
"		   # A larger mask size is better for larger images.\n"
"		   # 3 pixels is enough for 640x480 images.\n"
"                   # Increasing this value makes the tracking slower.\n"
"		   #\n"
"		   # Caution: this value cannot be changed dynamically\n"
"		   # without resetting the tracking.\n"
"\n"
"int64 range        # Maximum seek distance on both sides of the reference pixel.\n"
"      		   # It should match the maximum distance in pixel between\n"
"		   # the current position of the feature projection and\n"
"		   # its next position.\n"
"		   # I.e. if the object moves fast and your tracking\n"
"		   # frequency is low, this value should be increased.\n"
"                   # Increasing this value makes the tracking slower.\n"
"\n"
"float64 threshold  # Value used to determine if a moving edge is valid\n"
"		   # or not.\n"
"\n"
"float64 mu1        # Minimum image contrast allowed to detect a contour.\n"
"float64 mu2        # Maximum image contrast allowed to detect a contour.\n"
"\n"
"int64 sample_step   # Minimum distance in pixel between two\n"
"      		    # discretization points.\n"
"      		    # It avoids having too many discretization points when\n"
"		    # the tracked object is far away (and its projection\n"
"		    # in the image is small).\n"
"		    # Increasing this value makes the tracking *faster*.\n"
"\n"
"int64 strip             # How many pixels are ignored around the borders.\n"
"\n"
"\n"
"# Tracker parameters.\n"
"\n"
"float64 first_threshold # What proportion of points should be valid to\n"
"                        # acccept an initial pose.\n"
"			# Value should be between 0 et 1.\n"
"\n"
;
  }

  static const char* value(const ::visp_tracker::MovingEdgeSettings_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::visp_tracker::MovingEdgeSettings_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.mask_size);
      stream.next(m.range);
      stream.next(m.threshold);
      stream.next(m.mu1);
      stream.next(m.mu2);
      stream.next(m.sample_step);
      stream.next(m.strip);
      stream.next(m.first_threshold);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MovingEdgeSettings_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::visp_tracker::MovingEdgeSettings_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::visp_tracker::MovingEdgeSettings_<ContainerAllocator>& v)
  {
    s << indent << "mask_size: ";
    Printer<int64_t>::stream(s, indent + "  ", v.mask_size);
    s << indent << "range: ";
    Printer<int64_t>::stream(s, indent + "  ", v.range);
    s << indent << "threshold: ";
    Printer<double>::stream(s, indent + "  ", v.threshold);
    s << indent << "mu1: ";
    Printer<double>::stream(s, indent + "  ", v.mu1);
    s << indent << "mu2: ";
    Printer<double>::stream(s, indent + "  ", v.mu2);
    s << indent << "sample_step: ";
    Printer<int64_t>::stream(s, indent + "  ", v.sample_step);
    s << indent << "strip: ";
    Printer<int64_t>::stream(s, indent + "  ", v.strip);
    s << indent << "first_threshold: ";
    Printer<double>::stream(s, indent + "  ", v.first_threshold);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VISP_TRACKER_MESSAGE_MOVINGEDGESETTINGS_H