// Generated by gencpp from file posedetection_msgs/TargetObjResponse.msg
// DO NOT EDIT!


#ifndef POSEDETECTION_MSGS_MESSAGE_TARGETOBJRESPONSE_H
#define POSEDETECTION_MSGS_MESSAGE_TARGETOBJRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <posedetection_msgs/Object6DPose.h>

namespace posedetection_msgs
{
template <class ContainerAllocator>
struct TargetObjResponse_
{
  typedef TargetObjResponse_<ContainerAllocator> Type;

  TargetObjResponse_()
    : object_pose()  {
    }
  TargetObjResponse_(const ContainerAllocator& _alloc)
    : object_pose(_alloc)  {
  (void)_alloc;
    }



   typedef  ::posedetection_msgs::Object6DPose_<ContainerAllocator>  _object_pose_type;
  _object_pose_type object_pose;





  typedef boost::shared_ptr< ::posedetection_msgs::TargetObjResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::posedetection_msgs::TargetObjResponse_<ContainerAllocator> const> ConstPtr;

}; // struct TargetObjResponse_

typedef ::posedetection_msgs::TargetObjResponse_<std::allocator<void> > TargetObjResponse;

typedef boost::shared_ptr< ::posedetection_msgs::TargetObjResponse > TargetObjResponsePtr;
typedef boost::shared_ptr< ::posedetection_msgs::TargetObjResponse const> TargetObjResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::posedetection_msgs::TargetObjResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::posedetection_msgs::TargetObjResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::posedetection_msgs::TargetObjResponse_<ContainerAllocator1> & lhs, const ::posedetection_msgs::TargetObjResponse_<ContainerAllocator2> & rhs)
{
  return lhs.object_pose == rhs.object_pose;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::posedetection_msgs::TargetObjResponse_<ContainerAllocator1> & lhs, const ::posedetection_msgs::TargetObjResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace posedetection_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::posedetection_msgs::TargetObjResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::posedetection_msgs::TargetObjResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::posedetection_msgs::TargetObjResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::posedetection_msgs::TargetObjResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::posedetection_msgs::TargetObjResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::posedetection_msgs::TargetObjResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::posedetection_msgs::TargetObjResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9e3e0d9a56ba420ae5c3854c1194abf0";
  }

  static const char* value(const ::posedetection_msgs::TargetObjResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9e3e0d9a56ba420aULL;
  static const uint64_t static_value2 = 0xe5c3854c1194abf0ULL;
};

template<class ContainerAllocator>
struct DataType< ::posedetection_msgs::TargetObjResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "posedetection_msgs/TargetObjResponse";
  }

  static const char* value(const ::posedetection_msgs::TargetObjResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::posedetection_msgs::TargetObjResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "posedetection_msgs/Object6DPose object_pose\n"
"\n"
"\n"
"================================================================================\n"
"MSG: posedetection_msgs/Object6DPose\n"
"# 6D pose of object\n"
"geometry_msgs/Pose pose\n"
"# reliability\n"
"float32 reliability\n"
"\n"
"# type of object, usually contains the filename of the object that allows the receiving side to visualize it\n"
"# can also be used as a unique type id\n"
"string type \n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::posedetection_msgs::TargetObjResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::posedetection_msgs::TargetObjResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.object_pose);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TargetObjResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::posedetection_msgs::TargetObjResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::posedetection_msgs::TargetObjResponse_<ContainerAllocator>& v)
  {
    s << indent << "object_pose: ";
    s << std::endl;
    Printer< ::posedetection_msgs::Object6DPose_<ContainerAllocator> >::stream(s, indent + "  ", v.object_pose);
  }
};

} // namespace message_operations
} // namespace ros

#endif // POSEDETECTION_MSGS_MESSAGE_TARGETOBJRESPONSE_H
