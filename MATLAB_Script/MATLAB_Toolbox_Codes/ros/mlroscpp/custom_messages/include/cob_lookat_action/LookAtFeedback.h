// Generated by gencpp from file cob_lookat_action/LookAtFeedback.msg
// DO NOT EDIT!


#ifndef COB_LOOKAT_ACTION_MESSAGE_LOOKATFEEDBACK_H
#define COB_LOOKAT_ACTION_MESSAGE_LOOKATFEEDBACK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace cob_lookat_action
{
template <class ContainerAllocator>
struct LookAtFeedback_
{
  typedef LookAtFeedback_<ContainerAllocator> Type;

  LookAtFeedback_()
    : status(false)
    , message()  {
    }
  LookAtFeedback_(const ContainerAllocator& _alloc)
    : status(false)
    , message(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _status_type;
  _status_type status;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _message_type;
  _message_type message;





  typedef boost::shared_ptr< ::cob_lookat_action::LookAtFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cob_lookat_action::LookAtFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct LookAtFeedback_

typedef ::cob_lookat_action::LookAtFeedback_<std::allocator<void> > LookAtFeedback;

typedef boost::shared_ptr< ::cob_lookat_action::LookAtFeedback > LookAtFeedbackPtr;
typedef boost::shared_ptr< ::cob_lookat_action::LookAtFeedback const> LookAtFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cob_lookat_action::LookAtFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cob_lookat_action::LookAtFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cob_lookat_action::LookAtFeedback_<ContainerAllocator1> & lhs, const ::cob_lookat_action::LookAtFeedback_<ContainerAllocator2> & rhs)
{
  return lhs.status == rhs.status &&
    lhs.message == rhs.message;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cob_lookat_action::LookAtFeedback_<ContainerAllocator1> & lhs, const ::cob_lookat_action::LookAtFeedback_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cob_lookat_action

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::cob_lookat_action::LookAtFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cob_lookat_action::LookAtFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cob_lookat_action::LookAtFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cob_lookat_action::LookAtFeedback_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cob_lookat_action::LookAtFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cob_lookat_action::LookAtFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cob_lookat_action::LookAtFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cc9c64df4628f5bb500ddeb635768626";
  }

  static const char* value(const ::cob_lookat_action::LookAtFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcc9c64df4628f5bbULL;
  static const uint64_t static_value2 = 0x500ddeb635768626ULL;
};

template<class ContainerAllocator>
struct DataType< ::cob_lookat_action::LookAtFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cob_lookat_action/LookAtFeedback";
  }

  static const char* value(const ::cob_lookat_action::LookAtFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cob_lookat_action::LookAtFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"#feedback\n"
"bool status\n"
"string message\n"
"\n"
;
  }

  static const char* value(const ::cob_lookat_action::LookAtFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cob_lookat_action::LookAtFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.status);
      stream.next(m.message);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LookAtFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cob_lookat_action::LookAtFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cob_lookat_action::LookAtFeedback_<ContainerAllocator>& v)
  {
    s << indent << "status: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.status);
    s << indent << "message: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.message);
  }
};

} // namespace message_operations
} // namespace ros

#endif // COB_LOOKAT_ACTION_MESSAGE_LOOKATFEEDBACK_H