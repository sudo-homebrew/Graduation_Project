// Generated by gencpp from file cob_script_server/ScriptResult.msg
// DO NOT EDIT!


#ifndef COB_SCRIPT_SERVER_MESSAGE_SCRIPTRESULT_H
#define COB_SCRIPT_SERVER_MESSAGE_SCRIPTRESULT_H


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
struct ScriptResult_
{
  typedef ScriptResult_<ContainerAllocator> Type;

  ScriptResult_()
    : error_code(0)  {
    }
  ScriptResult_(const ContainerAllocator& _alloc)
    : error_code(0)  {
  (void)_alloc;
    }



   typedef int32_t _error_code_type;
  _error_code_type error_code;





  typedef boost::shared_ptr< ::cob_script_server::ScriptResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cob_script_server::ScriptResult_<ContainerAllocator> const> ConstPtr;

}; // struct ScriptResult_

typedef ::cob_script_server::ScriptResult_<std::allocator<void> > ScriptResult;

typedef boost::shared_ptr< ::cob_script_server::ScriptResult > ScriptResultPtr;
typedef boost::shared_ptr< ::cob_script_server::ScriptResult const> ScriptResultConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cob_script_server::ScriptResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cob_script_server::ScriptResult_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cob_script_server::ScriptResult_<ContainerAllocator1> & lhs, const ::cob_script_server::ScriptResult_<ContainerAllocator2> & rhs)
{
  return lhs.error_code == rhs.error_code;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cob_script_server::ScriptResult_<ContainerAllocator1> & lhs, const ::cob_script_server::ScriptResult_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cob_script_server

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::cob_script_server::ScriptResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cob_script_server::ScriptResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cob_script_server::ScriptResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cob_script_server::ScriptResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cob_script_server::ScriptResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cob_script_server::ScriptResult_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cob_script_server::ScriptResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ea324a22c787839f822b9a025bc2c6fe";
  }

  static const char* value(const ::cob_script_server::ScriptResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xea324a22c787839fULL;
  static const uint64_t static_value2 = 0x822b9a025bc2c6feULL;
};

template<class ContainerAllocator>
struct DataType< ::cob_script_server::ScriptResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cob_script_server/ScriptResult";
  }

  static const char* value(const ::cob_script_server::ScriptResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cob_script_server::ScriptResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"#result definition\n"
"int32 error_code\n"
;
  }

  static const char* value(const ::cob_script_server::ScriptResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cob_script_server::ScriptResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.error_code);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ScriptResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cob_script_server::ScriptResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cob_script_server::ScriptResult_<ContainerAllocator>& v)
  {
    s << indent << "error_code: ";
    Printer<int32_t>::stream(s, indent + "  ", v.error_code);
  }
};

} // namespace message_operations
} // namespace ros

#endif // COB_SCRIPT_SERVER_MESSAGE_SCRIPTRESULT_H
