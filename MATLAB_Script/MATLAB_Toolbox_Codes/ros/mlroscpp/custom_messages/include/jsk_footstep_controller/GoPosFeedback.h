// Generated by gencpp from file jsk_footstep_controller/GoPosFeedback.msg
// DO NOT EDIT!


#ifndef JSK_FOOTSTEP_CONTROLLER_MESSAGE_GOPOSFEEDBACK_H
#define JSK_FOOTSTEP_CONTROLLER_MESSAGE_GOPOSFEEDBACK_H


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
struct GoPosFeedback_
{
  typedef GoPosFeedback_<ContainerAllocator> Type;

  GoPosFeedback_()
    : status(0)
    , remaining_steps(0)  {
    }
  GoPosFeedback_(const ContainerAllocator& _alloc)
    : status(0)
    , remaining_steps(0)  {
  (void)_alloc;
    }



   typedef uint8_t _status_type;
  _status_type status;

   typedef int32_t _remaining_steps_type;
  _remaining_steps_type remaining_steps;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(PRE_PLANNING)
  #undef PRE_PLANNING
#endif
#if defined(_WIN32) && defined(PLANNING)
  #undef PLANNING
#endif
#if defined(_WIN32) && defined(WALKING)
  #undef WALKING
#endif
#if defined(_WIN32) && defined(WAITING)
  #undef WAITING
#endif
#if defined(_WIN32) && defined(FINISH)
  #undef FINISH
#endif

  enum {
    PRE_PLANNING = 0u,
    PLANNING = 1u,
    WALKING = 2u,
    WAITING = 3u,
    FINISH = 4u,
  };


  typedef boost::shared_ptr< ::jsk_footstep_controller::GoPosFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_footstep_controller::GoPosFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct GoPosFeedback_

typedef ::jsk_footstep_controller::GoPosFeedback_<std::allocator<void> > GoPosFeedback;

typedef boost::shared_ptr< ::jsk_footstep_controller::GoPosFeedback > GoPosFeedbackPtr;
typedef boost::shared_ptr< ::jsk_footstep_controller::GoPosFeedback const> GoPosFeedbackConstPtr;

// constants requiring out of line definition

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_footstep_controller::GoPosFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_footstep_controller::GoPosFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::jsk_footstep_controller::GoPosFeedback_<ContainerAllocator1> & lhs, const ::jsk_footstep_controller::GoPosFeedback_<ContainerAllocator2> & rhs)
{
  return lhs.status == rhs.status &&
    lhs.remaining_steps == rhs.remaining_steps;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::jsk_footstep_controller::GoPosFeedback_<ContainerAllocator1> & lhs, const ::jsk_footstep_controller::GoPosFeedback_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace jsk_footstep_controller

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::jsk_footstep_controller::GoPosFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_footstep_controller::GoPosFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_footstep_controller::GoPosFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_footstep_controller::GoPosFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_footstep_controller::GoPosFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_footstep_controller::GoPosFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_footstep_controller::GoPosFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4b0fee2db6d7d95642702999456a0721";
  }

  static const char* value(const ::jsk_footstep_controller::GoPosFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4b0fee2db6d7d956ULL;
  static const uint64_t static_value2 = 0x42702999456a0721ULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_footstep_controller::GoPosFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_footstep_controller/GoPosFeedback";
  }

  static const char* value(const ::jsk_footstep_controller::GoPosFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_footstep_controller::GoPosFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"# feedback\n"
"uint8 PRE_PLANNING=0\n"
"uint8 PLANNING=1\n"
"uint8 WALKING=2\n"
"uint8 WAITING=3\n"
"uint8 FINISH=4\n"
"uint8 status\n"
"int32 remaining_steps\n"
"\n"
;
  }

  static const char* value(const ::jsk_footstep_controller::GoPosFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_footstep_controller::GoPosFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.status);
      stream.next(m.remaining_steps);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GoPosFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_footstep_controller::GoPosFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_footstep_controller::GoPosFeedback_<ContainerAllocator>& v)
  {
    s << indent << "status: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.status);
    s << indent << "remaining_steps: ";
    Printer<int32_t>::stream(s, indent + "  ", v.remaining_steps);
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_FOOTSTEP_CONTROLLER_MESSAGE_GOPOSFEEDBACK_H