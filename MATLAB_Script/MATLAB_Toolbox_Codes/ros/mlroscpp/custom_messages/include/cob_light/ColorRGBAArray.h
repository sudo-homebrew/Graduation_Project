// Generated by gencpp from file cob_light/ColorRGBAArray.msg
// DO NOT EDIT!


#ifndef COB_LIGHT_MESSAGE_COLORRGBAARRAY_H
#define COB_LIGHT_MESSAGE_COLORRGBAARRAY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/ColorRGBA.h>

namespace cob_light
{
template <class ContainerAllocator>
struct ColorRGBAArray_
{
  typedef ColorRGBAArray_<ContainerAllocator> Type;

  ColorRGBAArray_()
    : colors()  {
    }
  ColorRGBAArray_(const ContainerAllocator& _alloc)
    : colors(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::std_msgs::ColorRGBA_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::std_msgs::ColorRGBA_<ContainerAllocator> >::other >  _colors_type;
  _colors_type colors;





  typedef boost::shared_ptr< ::cob_light::ColorRGBAArray_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cob_light::ColorRGBAArray_<ContainerAllocator> const> ConstPtr;

}; // struct ColorRGBAArray_

typedef ::cob_light::ColorRGBAArray_<std::allocator<void> > ColorRGBAArray;

typedef boost::shared_ptr< ::cob_light::ColorRGBAArray > ColorRGBAArrayPtr;
typedef boost::shared_ptr< ::cob_light::ColorRGBAArray const> ColorRGBAArrayConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cob_light::ColorRGBAArray_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cob_light::ColorRGBAArray_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cob_light::ColorRGBAArray_<ContainerAllocator1> & lhs, const ::cob_light::ColorRGBAArray_<ContainerAllocator2> & rhs)
{
  return lhs.colors == rhs.colors;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cob_light::ColorRGBAArray_<ContainerAllocator1> & lhs, const ::cob_light::ColorRGBAArray_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cob_light

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::cob_light::ColorRGBAArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cob_light::ColorRGBAArray_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cob_light::ColorRGBAArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cob_light::ColorRGBAArray_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cob_light::ColorRGBAArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cob_light::ColorRGBAArray_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cob_light::ColorRGBAArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8a8aae411a07648ba08dd6bedf519336";
  }

  static const char* value(const ::cob_light::ColorRGBAArray_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8a8aae411a07648bULL;
  static const uint64_t static_value2 = 0xa08dd6bedf519336ULL;
};

template<class ContainerAllocator>
struct DataType< ::cob_light::ColorRGBAArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cob_light/ColorRGBAArray";
  }

  static const char* value(const ::cob_light::ColorRGBAArray_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cob_light::ColorRGBAArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/ColorRGBA[] colors\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/ColorRGBA\n"
"float32 r\n"
"float32 g\n"
"float32 b\n"
"float32 a\n"
;
  }

  static const char* value(const ::cob_light::ColorRGBAArray_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cob_light::ColorRGBAArray_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.colors);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ColorRGBAArray_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cob_light::ColorRGBAArray_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cob_light::ColorRGBAArray_<ContainerAllocator>& v)
  {
    s << indent << "colors[]" << std::endl;
    for (size_t i = 0; i < v.colors.size(); ++i)
    {
      s << indent << "  colors[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::std_msgs::ColorRGBA_<ContainerAllocator> >::stream(s, indent + "    ", v.colors[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // COB_LIGHT_MESSAGE_COLORRGBAARRAY_H