// Generated by gencpp from file cob_grasp_generation/GenerateGraspsResult.msg
// DO NOT EDIT!


#ifndef COB_GRASP_GENERATION_MESSAGE_GENERATEGRASPSRESULT_H
#define COB_GRASP_GENERATION_MESSAGE_GENERATEGRASPSRESULT_H


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
struct GenerateGraspsResult_
{
  typedef GenerateGraspsResult_<ContainerAllocator> Type;

  GenerateGraspsResult_()
    : success(false)
    , num_grasps(0)  {
    }
  GenerateGraspsResult_(const ContainerAllocator& _alloc)
    : success(false)
    , num_grasps(0)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;

   typedef uint32_t _num_grasps_type;
  _num_grasps_type num_grasps;





  typedef boost::shared_ptr< ::cob_grasp_generation::GenerateGraspsResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cob_grasp_generation::GenerateGraspsResult_<ContainerAllocator> const> ConstPtr;

}; // struct GenerateGraspsResult_

typedef ::cob_grasp_generation::GenerateGraspsResult_<std::allocator<void> > GenerateGraspsResult;

typedef boost::shared_ptr< ::cob_grasp_generation::GenerateGraspsResult > GenerateGraspsResultPtr;
typedef boost::shared_ptr< ::cob_grasp_generation::GenerateGraspsResult const> GenerateGraspsResultConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cob_grasp_generation::GenerateGraspsResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cob_grasp_generation::GenerateGraspsResult_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cob_grasp_generation::GenerateGraspsResult_<ContainerAllocator1> & lhs, const ::cob_grasp_generation::GenerateGraspsResult_<ContainerAllocator2> & rhs)
{
  return lhs.success == rhs.success &&
    lhs.num_grasps == rhs.num_grasps;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cob_grasp_generation::GenerateGraspsResult_<ContainerAllocator1> & lhs, const ::cob_grasp_generation::GenerateGraspsResult_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cob_grasp_generation

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::cob_grasp_generation::GenerateGraspsResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cob_grasp_generation::GenerateGraspsResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cob_grasp_generation::GenerateGraspsResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cob_grasp_generation::GenerateGraspsResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cob_grasp_generation::GenerateGraspsResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cob_grasp_generation::GenerateGraspsResult_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cob_grasp_generation::GenerateGraspsResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "89a93e15e1906977d5060def30025192";
  }

  static const char* value(const ::cob_grasp_generation::GenerateGraspsResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x89a93e15e1906977ULL;
  static const uint64_t static_value2 = 0xd5060def30025192ULL;
};

template<class ContainerAllocator>
struct DataType< ::cob_grasp_generation::GenerateGraspsResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cob_grasp_generation/GenerateGraspsResult";
  }

  static const char* value(const ::cob_grasp_generation::GenerateGraspsResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cob_grasp_generation::GenerateGraspsResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"#result definition\n"
"bool success\n"
"uint32 num_grasps\n"
;
  }

  static const char* value(const ::cob_grasp_generation::GenerateGraspsResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cob_grasp_generation::GenerateGraspsResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
      stream.next(m.num_grasps);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GenerateGraspsResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cob_grasp_generation::GenerateGraspsResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cob_grasp_generation::GenerateGraspsResult_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
    s << indent << "num_grasps: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.num_grasps);
  }
};

} // namespace message_operations
} // namespace ros

#endif // COB_GRASP_GENERATION_MESSAGE_GENERATEGRASPSRESULT_H