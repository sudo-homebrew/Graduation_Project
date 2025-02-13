// Generated by gencpp from file jsk_footstep_controller/LookAroundGroundGoal.msg
// DO NOT EDIT!


#ifndef JSK_FOOTSTEP_CONTROLLER_MESSAGE_LOOKAROUNDGROUNDGOAL_H
#define JSK_FOOTSTEP_CONTROLLER_MESSAGE_LOOKAROUNDGROUNDGOAL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace jsk_footstep_controller
{
template <class ContainerAllocator>
struct LookAroundGroundGoal_
{
  typedef LookAroundGroundGoal_<ContainerAllocator> Type;

  LookAroundGroundGoal_()
    {
    }
  LookAroundGroundGoal_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::jsk_footstep_controller::LookAroundGroundGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_footstep_controller::LookAroundGroundGoal_<ContainerAllocator> const> ConstPtr;

}; // struct LookAroundGroundGoal_

typedef ::jsk_footstep_controller::LookAroundGroundGoal_<std::allocator<void> > LookAroundGroundGoal;

typedef boost::shared_ptr< ::jsk_footstep_controller::LookAroundGroundGoal > LookAroundGroundGoalPtr;
typedef boost::shared_ptr< ::jsk_footstep_controller::LookAroundGroundGoal const> LookAroundGroundGoalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_footstep_controller::LookAroundGroundGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_footstep_controller::LookAroundGroundGoal_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace jsk_footstep_controller

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::jsk_footstep_controller::LookAroundGroundGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_footstep_controller::LookAroundGroundGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_footstep_controller::LookAroundGroundGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_footstep_controller::LookAroundGroundGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_footstep_controller::LookAroundGroundGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_footstep_controller::LookAroundGroundGoal_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_footstep_controller::LookAroundGroundGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::jsk_footstep_controller::LookAroundGroundGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_footstep_controller::LookAroundGroundGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_footstep_controller/LookAroundGroundGoal";
  }

  static const char* value(const ::jsk_footstep_controller::LookAroundGroundGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_footstep_controller::LookAroundGroundGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"# goal\n"
;
  }

  static const char* value(const ::jsk_footstep_controller::LookAroundGroundGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_footstep_controller::LookAroundGroundGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LookAroundGroundGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_footstep_controller::LookAroundGroundGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::jsk_footstep_controller::LookAroundGroundGoal_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // JSK_FOOTSTEP_CONTROLLER_MESSAGE_LOOKAROUNDGROUNDGOAL_H
