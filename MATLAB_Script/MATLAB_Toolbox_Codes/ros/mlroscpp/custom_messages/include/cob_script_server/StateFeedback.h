// Generated by gencpp from file cob_script_server/StateFeedback.msg
// DO NOT EDIT!


#ifndef COB_SCRIPT_SERVER_MESSAGE_STATEFEEDBACK_H
#define COB_SCRIPT_SERVER_MESSAGE_STATEFEEDBACK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace cob_script_server
{
template <class ContainerAllocator>
struct StateFeedback_
{
  typedef StateFeedback_<ContainerAllocator> Type;

  StateFeedback_()
    {
    }
  StateFeedback_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::cob_script_server::StateFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cob_script_server::StateFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct StateFeedback_

typedef ::cob_script_server::StateFeedback_<std::allocator<void> > StateFeedback;

typedef boost::shared_ptr< ::cob_script_server::StateFeedback > StateFeedbackPtr;
typedef boost::shared_ptr< ::cob_script_server::StateFeedback const> StateFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cob_script_server::StateFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cob_script_server::StateFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace cob_script_server

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::cob_script_server::StateFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cob_script_server::StateFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cob_script_server::StateFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cob_script_server::StateFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cob_script_server::StateFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cob_script_server::StateFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cob_script_server::StateFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::cob_script_server::StateFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::cob_script_server::StateFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cob_script_server/StateFeedback";
  }

  static const char* value(const ::cob_script_server::StateFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cob_script_server::StateFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"#feedback definition\n"
"\n"
;
  }

  static const char* value(const ::cob_script_server::StateFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cob_script_server::StateFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct StateFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cob_script_server::StateFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::cob_script_server::StateFeedback_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // COB_SCRIPT_SERVER_MESSAGE_STATEFEEDBACK_H