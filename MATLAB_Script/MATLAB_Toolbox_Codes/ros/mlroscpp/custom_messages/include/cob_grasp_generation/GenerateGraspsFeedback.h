// Generated by gencpp from file cob_grasp_generation/GenerateGraspsFeedback.msg
// DO NOT EDIT!


#ifndef COB_GRASP_GENERATION_MESSAGE_GENERATEGRASPSFEEDBACK_H
#define COB_GRASP_GENERATION_MESSAGE_GENERATEGRASPSFEEDBACK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace cob_grasp_generation
{
template <class ContainerAllocator>
struct GenerateGraspsFeedback_
{
  typedef GenerateGraspsFeedback_<ContainerAllocator> Type;

  GenerateGraspsFeedback_()
    : status(false)  {
    }
  GenerateGraspsFeedback_(const ContainerAllocator& _alloc)
    : status(false)  {
  (void)_alloc;
    }



   typedef uint8_t _status_type;
  _status_type status;





  typedef boost::shared_ptr< ::cob_grasp_generation::GenerateGraspsFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cob_grasp_generation::GenerateGraspsFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct GenerateGraspsFeedback_

typedef ::cob_grasp_generation::GenerateGraspsFeedback_<std::allocator<void> > GenerateGraspsFeedback;

typedef boost::shared_ptr< ::cob_grasp_generation::GenerateGraspsFeedback > GenerateGraspsFeedbackPtr;
typedef boost::shared_ptr< ::cob_grasp_generation::GenerateGraspsFeedback const> GenerateGraspsFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cob_grasp_generation::GenerateGraspsFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cob_grasp_generation::GenerateGraspsFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cob_grasp_generation::GenerateGraspsFeedback_<ContainerAllocator1> & lhs, const ::cob_grasp_generation::GenerateGraspsFeedback_<ContainerAllocator2> & rhs)
{
  return lhs.status == rhs.status;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cob_grasp_generation::GenerateGraspsFeedback_<ContainerAllocator1> & lhs, const ::cob_grasp_generation::GenerateGraspsFeedback_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cob_grasp_generation

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::cob_grasp_generation::GenerateGraspsFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cob_grasp_generation::GenerateGraspsFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cob_grasp_generation::GenerateGraspsFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cob_grasp_generation::GenerateGraspsFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cob_grasp_generation::GenerateGraspsFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cob_grasp_generation::GenerateGraspsFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cob_grasp_generation::GenerateGraspsFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3a1255d4d998bd4d6585c64639b5ee9a";
  }

  static const char* value(const ::cob_grasp_generation::GenerateGraspsFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3a1255d4d998bd4dULL;
  static const uint64_t static_value2 = 0x6585c64639b5ee9aULL;
};

template<class ContainerAllocator>
struct DataType< ::cob_grasp_generation::GenerateGraspsFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cob_grasp_generation/GenerateGraspsFeedback";
  }

  static const char* value(const ::cob_grasp_generation::GenerateGraspsFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cob_grasp_generation::GenerateGraspsFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"#feedback\n"
"bool status\n"
"\n"
;
  }

  static const char* value(const ::cob_grasp_generation::GenerateGraspsFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cob_grasp_generation::GenerateGraspsFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GenerateGraspsFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cob_grasp_generation::GenerateGraspsFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cob_grasp_generation::GenerateGraspsFeedback_<ContainerAllocator>& v)
  {
    s << indent << "status: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.status);
  }
};

} // namespace message_operations
} // namespace ros

#endif // COB_GRASP_GENERATION_MESSAGE_GENERATEGRASPSFEEDBACK_H