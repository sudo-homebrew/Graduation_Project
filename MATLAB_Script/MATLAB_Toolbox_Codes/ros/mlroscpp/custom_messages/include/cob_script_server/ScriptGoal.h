// Generated by gencpp from file cob_script_server/ScriptGoal.msg
// DO NOT EDIT!


#ifndef COB_SCRIPT_SERVER_MESSAGE_SCRIPTGOAL_H
#define COB_SCRIPT_SERVER_MESSAGE_SCRIPTGOAL_H


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
struct ScriptGoal_
{
  typedef ScriptGoal_<ContainerAllocator> Type;

  ScriptGoal_()
    : function_name()
    , component_name()
    , parameter_name()
    , mode()
    , service_name()
    , duration(0.0)
    , planning(false)  {
    }
  ScriptGoal_(const ContainerAllocator& _alloc)
    : function_name(_alloc)
    , component_name(_alloc)
    , parameter_name(_alloc)
    , mode(_alloc)
    , service_name(_alloc)
    , duration(0.0)
    , planning(false)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _function_name_type;
  _function_name_type function_name;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _component_name_type;
  _component_name_type component_name;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _parameter_name_type;
  _parameter_name_type parameter_name;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _mode_type;
  _mode_type mode;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _service_name_type;
  _service_name_type service_name;

   typedef float _duration_type;
  _duration_type duration;

   typedef uint8_t _planning_type;
  _planning_type planning;





  typedef boost::shared_ptr< ::cob_script_server::ScriptGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cob_script_server::ScriptGoal_<ContainerAllocator> const> ConstPtr;

}; // struct ScriptGoal_

typedef ::cob_script_server::ScriptGoal_<std::allocator<void> > ScriptGoal;

typedef boost::shared_ptr< ::cob_script_server::ScriptGoal > ScriptGoalPtr;
typedef boost::shared_ptr< ::cob_script_server::ScriptGoal const> ScriptGoalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cob_script_server::ScriptGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cob_script_server::ScriptGoal_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cob_script_server::ScriptGoal_<ContainerAllocator1> & lhs, const ::cob_script_server::ScriptGoal_<ContainerAllocator2> & rhs)
{
  return lhs.function_name == rhs.function_name &&
    lhs.component_name == rhs.component_name &&
    lhs.parameter_name == rhs.parameter_name &&
    lhs.mode == rhs.mode &&
    lhs.service_name == rhs.service_name &&
    lhs.duration == rhs.duration &&
    lhs.planning == rhs.planning;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cob_script_server::ScriptGoal_<ContainerAllocator1> & lhs, const ::cob_script_server::ScriptGoal_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cob_script_server

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::cob_script_server::ScriptGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cob_script_server::ScriptGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cob_script_server::ScriptGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cob_script_server::ScriptGoal_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cob_script_server::ScriptGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cob_script_server::ScriptGoal_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cob_script_server::ScriptGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0a40a194b72e6783b1b8bec8d1c28c7f";
  }

  static const char* value(const ::cob_script_server::ScriptGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0a40a194b72e6783ULL;
  static const uint64_t static_value2 = 0xb1b8bec8d1c28c7fULL;
};

template<class ContainerAllocator>
struct DataType< ::cob_script_server::ScriptGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cob_script_server/ScriptGoal";
  }

  static const char* value(const ::cob_script_server::ScriptGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cob_script_server::ScriptGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"#goal definition\n"
"string function_name\n"
"string component_name\n"
"string parameter_name\n"
"string mode\n"
"string service_name\n"
"float32 duration\n"
"bool planning\n"
;
  }

  static const char* value(const ::cob_script_server::ScriptGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cob_script_server::ScriptGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.function_name);
      stream.next(m.component_name);
      stream.next(m.parameter_name);
      stream.next(m.mode);
      stream.next(m.service_name);
      stream.next(m.duration);
      stream.next(m.planning);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ScriptGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cob_script_server::ScriptGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cob_script_server::ScriptGoal_<ContainerAllocator>& v)
  {
    s << indent << "function_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.function_name);
    s << indent << "component_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.component_name);
    s << indent << "parameter_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.parameter_name);
    s << indent << "mode: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.mode);
    s << indent << "service_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.service_name);
    s << indent << "duration: ";
    Printer<float>::stream(s, indent + "  ", v.duration);
    s << indent << "planning: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.planning);
  }
};

} // namespace message_operations
} // namespace ros

#endif // COB_SCRIPT_SERVER_MESSAGE_SCRIPTGOAL_H