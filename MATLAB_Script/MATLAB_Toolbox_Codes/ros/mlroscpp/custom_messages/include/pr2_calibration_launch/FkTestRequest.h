// Generated by gencpp from file pr2_calibration_launch/FkTestRequest.msg
// DO NOT EDIT!


#ifndef PR2_CALIBRATION_LAUNCH_MESSAGE_FKTESTREQUEST_H
#define PR2_CALIBRATION_LAUNCH_MESSAGE_FKTESTREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace pr2_calibration_launch
{
template <class ContainerAllocator>
struct FkTestRequest_
{
  typedef FkTestRequest_<ContainerAllocator> Type;

  FkTestRequest_()
    : root()
    , tip()
    , joint_positions()  {
    }
  FkTestRequest_(const ContainerAllocator& _alloc)
    : root(_alloc)
    , tip(_alloc)
    , joint_positions(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _root_type;
  _root_type root;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _tip_type;
  _tip_type tip;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _joint_positions_type;
  _joint_positions_type joint_positions;





  typedef boost::shared_ptr< ::pr2_calibration_launch::FkTestRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pr2_calibration_launch::FkTestRequest_<ContainerAllocator> const> ConstPtr;

}; // struct FkTestRequest_

typedef ::pr2_calibration_launch::FkTestRequest_<std::allocator<void> > FkTestRequest;

typedef boost::shared_ptr< ::pr2_calibration_launch::FkTestRequest > FkTestRequestPtr;
typedef boost::shared_ptr< ::pr2_calibration_launch::FkTestRequest const> FkTestRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pr2_calibration_launch::FkTestRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pr2_calibration_launch::FkTestRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace pr2_calibration_launch

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1339814/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::pr2_calibration_launch::FkTestRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pr2_calibration_launch::FkTestRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pr2_calibration_launch::FkTestRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pr2_calibration_launch::FkTestRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pr2_calibration_launch::FkTestRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pr2_calibration_launch::FkTestRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pr2_calibration_launch::FkTestRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "708e14f98ff72822d3442bcaef9c218d";
  }

  static const char* value(const ::pr2_calibration_launch::FkTestRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x708e14f98ff72822ULL;
  static const uint64_t static_value2 = 0xd3442bcaef9c218dULL;
};

template<class ContainerAllocator>
struct DataType< ::pr2_calibration_launch::FkTestRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pr2_calibration_launch/FkTestRequest";
  }

  static const char* value(const ::pr2_calibration_launch::FkTestRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pr2_calibration_launch::FkTestRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string root\n"
"string tip\n"
"float64[] joint_positions\n"
;
  }

  static const char* value(const ::pr2_calibration_launch::FkTestRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pr2_calibration_launch::FkTestRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.root);
      stream.next(m.tip);
      stream.next(m.joint_positions);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FkTestRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pr2_calibration_launch::FkTestRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pr2_calibration_launch::FkTestRequest_<ContainerAllocator>& v)
  {
    s << indent << "root: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.root);
    s << indent << "tip: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.tip);
    s << indent << "joint_positions[]" << std::endl;
    for (size_t i = 0; i < v.joint_positions.size(); ++i)
    {
      s << indent << "  joint_positions[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.joint_positions[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // PR2_CALIBRATION_LAUNCH_MESSAGE_FKTESTREQUEST_H