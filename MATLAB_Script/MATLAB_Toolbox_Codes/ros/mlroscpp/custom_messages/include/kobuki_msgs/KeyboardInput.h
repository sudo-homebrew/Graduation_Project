// Generated by gencpp from file kobuki_msgs/KeyboardInput.msg
// DO NOT EDIT!


#ifndef KOBUKI_MSGS_MESSAGE_KEYBOARDINPUT_H
#define KOBUKI_MSGS_MESSAGE_KEYBOARDINPUT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace kobuki_msgs
{
template <class ContainerAllocator>
struct KeyboardInput_
{
  typedef KeyboardInput_<ContainerAllocator> Type;

  KeyboardInput_()
    : pressed_key(0)  {
    }
  KeyboardInput_(const ContainerAllocator& _alloc)
    : pressed_key(0)  {
  (void)_alloc;
    }



   typedef uint8_t _pressed_key_type;
  _pressed_key_type pressed_key;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(KEYCODE_RIGHT)
  #undef KEYCODE_RIGHT
#endif
#if defined(_WIN32) && defined(KEYCODE_LEFT)
  #undef KEYCODE_LEFT
#endif
#if defined(_WIN32) && defined(KEYCODE_UP)
  #undef KEYCODE_UP
#endif
#if defined(_WIN32) && defined(KEYCODE_DOWN)
  #undef KEYCODE_DOWN
#endif
#if defined(_WIN32) && defined(KEYCODE_SPACE)
  #undef KEYCODE_SPACE
#endif
#if defined(_WIN32) && defined(KEYCODE_ENABLE)
  #undef KEYCODE_ENABLE
#endif
#if defined(_WIN32) && defined(KEYCODE_DISABLE)
  #undef KEYCODE_DISABLE
#endif

  enum {
    KEYCODE_RIGHT = 67u,
    KEYCODE_LEFT = 68u,
    KEYCODE_UP = 65u,
    KEYCODE_DOWN = 66u,
    KEYCODE_SPACE = 32u,
    KEYCODE_ENABLE = 101u,
    KEYCODE_DISABLE = 100u,
  };


  typedef boost::shared_ptr< ::kobuki_msgs::KeyboardInput_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kobuki_msgs::KeyboardInput_<ContainerAllocator> const> ConstPtr;

}; // struct KeyboardInput_

typedef ::kobuki_msgs::KeyboardInput_<std::allocator<void> > KeyboardInput;

typedef boost::shared_ptr< ::kobuki_msgs::KeyboardInput > KeyboardInputPtr;
typedef boost::shared_ptr< ::kobuki_msgs::KeyboardInput const> KeyboardInputConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kobuki_msgs::KeyboardInput_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kobuki_msgs::KeyboardInput_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kobuki_msgs::KeyboardInput_<ContainerAllocator1> & lhs, const ::kobuki_msgs::KeyboardInput_<ContainerAllocator2> & rhs)
{
  return lhs.pressed_key == rhs.pressed_key;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kobuki_msgs::KeyboardInput_<ContainerAllocator1> & lhs, const ::kobuki_msgs::KeyboardInput_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kobuki_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kobuki_msgs::KeyboardInput_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kobuki_msgs::KeyboardInput_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kobuki_msgs::KeyboardInput_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kobuki_msgs::KeyboardInput_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kobuki_msgs::KeyboardInput_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kobuki_msgs::KeyboardInput_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kobuki_msgs::KeyboardInput_<ContainerAllocator> >
{
  static const char* value()
  {
    return "abfdba4ea859b317d6aa3626aff1e35e";
  }

  static const char* value(const ::kobuki_msgs::KeyboardInput_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xabfdba4ea859b317ULL;
  static const uint64_t static_value2 = 0xd6aa3626aff1e35eULL;
};

template<class ContainerAllocator>
struct DataType< ::kobuki_msgs::KeyboardInput_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kobuki_msgs/KeyboardInput";
  }

  static const char* value(const ::kobuki_msgs::KeyboardInput_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kobuki_msgs::KeyboardInput_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# KEYBOARD INPUT\n"
"# \n"
"# Keycodes to be transferred for remote teleops.\n"
"\n"
"uint8  KEYCODE_RIGHT    = 67     # 0x43\n"
"uint8  KEYCODE_LEFT     = 68     # 0x44\n"
"uint8  KEYCODE_UP       = 65     # 0x41\n"
"uint8  KEYCODE_DOWN     = 66     # 0x42\n"
"uint8  KEYCODE_SPACE    = 32     # 0x20\n"
"uint8  KEYCODE_ENABLE   = 101    # 0x65, 'e'\n"
"uint8  KEYCODE_DISABLE  = 100    # 0x64, 'd'\n"
"\n"
"uint8 pressed_key\n"
;
  }

  static const char* value(const ::kobuki_msgs::KeyboardInput_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kobuki_msgs::KeyboardInput_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pressed_key);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct KeyboardInput_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kobuki_msgs::KeyboardInput_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kobuki_msgs::KeyboardInput_<ContainerAllocator>& v)
  {
    s << indent << "pressed_key: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.pressed_key);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KOBUKI_MSGS_MESSAGE_KEYBOARDINPUT_H