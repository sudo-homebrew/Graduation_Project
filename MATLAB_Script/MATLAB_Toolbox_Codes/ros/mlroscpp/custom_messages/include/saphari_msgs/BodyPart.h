// Generated by gencpp from file saphari_msgs/BodyPart.msg
// DO NOT EDIT!


#ifndef SAPHARI_MSGS_MESSAGE_BODYPART_H
#define SAPHARI_MSGS_MESSAGE_BODYPART_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point32.h>

namespace saphari_msgs
{
template <class ContainerAllocator>
struct BodyPart_
{
  typedef BodyPart_<ContainerAllocator> Type;

  BodyPart_()
    : id(0)
    , label(0)
    , childIDs()
    , centroid()
    , radius(0.0)  {
    }
  BodyPart_(const ContainerAllocator& _alloc)
    : id(0)
    , label(0)
    , childIDs(_alloc)
    , centroid(_alloc)
    , radius(0.0)  {
  (void)_alloc;
    }



   typedef int32_t _id_type;
  _id_type id;

   typedef int32_t _label_type;
  _label_type label;

   typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _childIDs_type;
  _childIDs_type childIDs;

   typedef  ::geometry_msgs::Point32_<ContainerAllocator>  _centroid_type;
  _centroid_type centroid;

   typedef float _radius_type;
  _radius_type radius;



  enum {
    LEFTFOOT = 0u,
    LEFTLEG = 1u,
    LEFTKNEE = 2u,
    LEFTTHIGH = 3u,
    RIGHTFOOT = 4u,
    RIGHTLEG = 5u,
    RIGHTKNEE = 6u,
    RIGHTTHIGH = 7u,
    RIGHTHIP = 8u,
    LEFTHIP = 9u,
    NECK = 10u,
    RIGHTARM = 11u,
    RIGHTELBOW = 12u,
    RIGHTFOREARM = 13u,
    RIGHTHAND = 14u,
    LEFTARM = 15u,
    LEFTELBOW = 16u,
    FOREARM = 17u,
    LEFTHAND = 18u,
    LEFTBOTTOMFACE = 19u,
    RIGHTBOTTOMFACE = 20u,
    LEFTTOPFACE = 21u,
    RIGHTTOPFACE = 22u,
    RIGHTCHEST = 23u,
    LEFTCHEST = 24u,
    HEAD = 96u,
    TORSO = 97u,
    RIGHTSHOULDER = 98u,
    LEFTSHOULDER = 99u,
  };


  typedef boost::shared_ptr< ::saphari_msgs::BodyPart_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::saphari_msgs::BodyPart_<ContainerAllocator> const> ConstPtr;

}; // struct BodyPart_

typedef ::saphari_msgs::BodyPart_<std::allocator<void> > BodyPart;

typedef boost::shared_ptr< ::saphari_msgs::BodyPart > BodyPartPtr;
typedef boost::shared_ptr< ::saphari_msgs::BodyPart const> BodyPartConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::saphari_msgs::BodyPart_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::saphari_msgs::BodyPart_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace saphari_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/geometry_msgs/msg'], 'saphari_msgs': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/saphari_msgs/msg'], 'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1329247.2/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg', '/mathworks/devel/sbs/31/pmurali.Brobot.j1329247.2/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::saphari_msgs::BodyPart_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::saphari_msgs::BodyPart_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::saphari_msgs::BodyPart_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::saphari_msgs::BodyPart_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::saphari_msgs::BodyPart_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::saphari_msgs::BodyPart_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::saphari_msgs::BodyPart_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f97d66c4403cdacb1c743e356fc26b62";
  }

  static const char* value(const ::saphari_msgs::BodyPart_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf97d66c4403cdacbULL;
  static const uint64_t static_value2 = 0x1c743e356fc26b62ULL;
};

template<class ContainerAllocator>
struct DataType< ::saphari_msgs::BodyPart_<ContainerAllocator> >
{
  static const char* value()
  {
    return "saphari_msgs/BodyPart";
  }

  static const char* value(const ::saphari_msgs::BodyPart_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::saphari_msgs::BodyPart_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 LEFTFOOT=0\n"
"uint8 LEFTLEG=1\n"
"uint8 LEFTKNEE=2\n"
"uint8 LEFTTHIGH=3\n"
"uint8 RIGHTFOOT=4\n"
"uint8 RIGHTLEG=5\n"
"uint8 RIGHTKNEE=6\n"
"uint8 RIGHTTHIGH=7\n"
"uint8 RIGHTHIP=8\n"
"uint8 LEFTHIP=9\n"
"uint8 NECK=10\n"
"uint8 RIGHTARM=11\n"
"uint8 RIGHTELBOW=12\n"
"uint8 RIGHTFOREARM=13\n"
"uint8 RIGHTHAND=14\n"
"uint8 LEFTARM=15\n"
"uint8 LEFTELBOW=16\n"
"uint8 FOREARM=17\n"
"uint8 LEFTHAND=18\n"
"uint8 LEFTBOTTOMFACE=19\n"
"uint8 RIGHTBOTTOMFACE=20\n"
"uint8 LEFTTOPFACE=21\n"
"uint8 RIGHTTOPFACE=22\n"
"uint8 RIGHTCHEST=23\n"
"uint8 LEFTCHEST=24\n"
"\n"
"#following ids are the backup for the demo using the ni tracker\n"
"uint8 HEAD=96\n"
"uint8 TORSO=97\n"
"uint8 RIGHTSHOULDER=98\n"
"uint8 LEFTSHOULDER=99\n"
"\n"
"int32 id\n"
"int32 label\n"
"int32[] childIDs\n"
"geometry_msgs/Point32 centroid\n"
"float32 radius\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point32\n"
"# This contains the position of a point in free space(with 32 bits of precision).\n"
"# It is recommeded to use Point wherever possible instead of Point32.  \n"
"# \n"
"# This recommendation is to promote interoperability.  \n"
"#\n"
"# This message is designed to take up less space when sending\n"
"# lots of points at once, as in the case of a PointCloud.  \n"
"\n"
"float32 x\n"
"float32 y\n"
"float32 z\n"
;
  }

  static const char* value(const ::saphari_msgs::BodyPart_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::saphari_msgs::BodyPart_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.label);
      stream.next(m.childIDs);
      stream.next(m.centroid);
      stream.next(m.radius);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct BodyPart_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::saphari_msgs::BodyPart_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::saphari_msgs::BodyPart_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.id);
    s << indent << "label: ";
    Printer<int32_t>::stream(s, indent + "  ", v.label);
    s << indent << "childIDs[]" << std::endl;
    for (size_t i = 0; i < v.childIDs.size(); ++i)
    {
      s << indent << "  childIDs[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.childIDs[i]);
    }
    s << indent << "centroid: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point32_<ContainerAllocator> >::stream(s, indent + "  ", v.centroid);
    s << indent << "radius: ";
    Printer<float>::stream(s, indent + "  ", v.radius);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SAPHARI_MSGS_MESSAGE_BODYPART_H