// Generated by gencpp from file bride_tutorials/TriggerPublishGoal.msg
// DO NOT EDIT!


#ifndef BRIDE_TUTORIALS_MESSAGE_TRIGGERPUBLISHGOAL_H
#define BRIDE_TUTORIALS_MESSAGE_TRIGGERPUBLISHGOAL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace bride_tutorials
{
template <class ContainerAllocator>
struct TriggerPublishGoal_
{
  typedef TriggerPublishGoal_<ContainerAllocator> Type;

  TriggerPublishGoal_()
    : test(0)  {
    }
  TriggerPublishGoal_(const ContainerAllocator& _alloc)
    : test(0)  {
  (void)_alloc;
    }



   typedef uint32_t _test_type;
  _test_type test;





  typedef boost::shared_ptr< ::bride_tutorials::TriggerPublishGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::bride_tutorials::TriggerPublishGoal_<ContainerAllocator> const> ConstPtr;

}; // struct TriggerPublishGoal_

typedef ::bride_tutorials::TriggerPublishGoal_<std::allocator<void> > TriggerPublishGoal;

typedef boost::shared_ptr< ::bride_tutorials::TriggerPublishGoal > TriggerPublishGoalPtr;
typedef boost::shared_ptr< ::bride_tutorials::TriggerPublishGoal const> TriggerPublishGoalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::bride_tutorials::TriggerPublishGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::bride_tutorials::TriggerPublishGoal_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace bride_tutorials

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'actionlib': ['/mathworks/devel/sbs/31/hakakarl.Brobot.j1347395/matlab/sys/ros1/glnxa64/ros1/share/actionlib/cmake/../msg'], 'std_msgs': ['/mathworks/devel/sbs/31/hakakarl.Brobot.j1347395/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg', '/mathworks/devel/sbs/31/hakakarl.Brobot.j1347395/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/mathworks/devel/sbs/31/hakakarl.Brobot.j1347395/matlab/sys/ros1/glnxa64/ros1/share/actionlib_msgs/cmake/../msg'], 'bride_tutorials': ['/local-ssd1/All_Custom_Msgs/PendingPackagesInitial/matlab_msg_gen_ros1/glnxa64/src/bride_tutorials/msg', '/local-ssd1/All_Custom_Msgs/PendingPackagesInitial/matlab_msg_gen_ros1/glnxa64/devel/share/bride_tutorials/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::bride_tutorials::TriggerPublishGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bride_tutorials::TriggerPublishGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bride_tutorials::TriggerPublishGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bride_tutorials::TriggerPublishGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bride_tutorials::TriggerPublishGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bride_tutorials::TriggerPublishGoal_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::bride_tutorials::TriggerPublishGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b66fa90afc91116f90d9ebb7313af521";
  }

  static const char* value(const ::bride_tutorials::TriggerPublishGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb66fa90afc91116fULL;
  static const uint64_t static_value2 = 0x90d9ebb7313af521ULL;
};

template<class ContainerAllocator>
struct DataType< ::bride_tutorials::TriggerPublishGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bride_tutorials/TriggerPublishGoal";
  }

  static const char* value(const ::bride_tutorials::TriggerPublishGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::bride_tutorials::TriggerPublishGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"# Define the goal\n"
"uint32 test\n"
;
  }

  static const char* value(const ::bride_tutorials::TriggerPublishGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::bride_tutorials::TriggerPublishGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.test);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TriggerPublishGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::bride_tutorials::TriggerPublishGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::bride_tutorials::TriggerPublishGoal_<ContainerAllocator>& v)
  {
    s << indent << "test: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.test);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BRIDE_TUTORIALS_MESSAGE_TRIGGERPUBLISHGOAL_H
