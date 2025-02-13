// Generated by gencpp from file jsk_rviz_plugins/RequestMarkerOperateResponse.msg
// DO NOT EDIT!


#ifndef JSK_RVIZ_PLUGINS_MESSAGE_REQUESTMARKEROPERATERESPONSE_H
#define JSK_RVIZ_PLUGINS_MESSAGE_REQUESTMARKEROPERATERESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace jsk_rviz_plugins
{
template <class ContainerAllocator>
struct RequestMarkerOperateResponse_
{
  typedef RequestMarkerOperateResponse_<ContainerAllocator> Type;

  RequestMarkerOperateResponse_()
    {
    }
  RequestMarkerOperateResponse_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::jsk_rviz_plugins::RequestMarkerOperateResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_rviz_plugins::RequestMarkerOperateResponse_<ContainerAllocator> const> ConstPtr;

}; // struct RequestMarkerOperateResponse_

typedef ::jsk_rviz_plugins::RequestMarkerOperateResponse_<std::allocator<void> > RequestMarkerOperateResponse;

typedef boost::shared_ptr< ::jsk_rviz_plugins::RequestMarkerOperateResponse > RequestMarkerOperateResponsePtr;
typedef boost::shared_ptr< ::jsk_rviz_plugins::RequestMarkerOperateResponse const> RequestMarkerOperateResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_rviz_plugins::RequestMarkerOperateResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_rviz_plugins::RequestMarkerOperateResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace jsk_rviz_plugins

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::jsk_rviz_plugins::RequestMarkerOperateResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_rviz_plugins::RequestMarkerOperateResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_rviz_plugins::RequestMarkerOperateResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_rviz_plugins::RequestMarkerOperateResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_rviz_plugins::RequestMarkerOperateResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_rviz_plugins::RequestMarkerOperateResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_rviz_plugins::RequestMarkerOperateResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::jsk_rviz_plugins::RequestMarkerOperateResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_rviz_plugins::RequestMarkerOperateResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_rviz_plugins/RequestMarkerOperateResponse";
  }

  static const char* value(const ::jsk_rviz_plugins::RequestMarkerOperateResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_rviz_plugins::RequestMarkerOperateResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::jsk_rviz_plugins::RequestMarkerOperateResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_rviz_plugins::RequestMarkerOperateResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RequestMarkerOperateResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_rviz_plugins::RequestMarkerOperateResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::jsk_rviz_plugins::RequestMarkerOperateResponse_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // JSK_RVIZ_PLUGINS_MESSAGE_REQUESTMARKEROPERATERESPONSE_H
