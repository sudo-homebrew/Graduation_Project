// Generated by gencpp from file concert_msgs/Implementation.msg
// DO NOT EDIT!


#ifndef CONCERT_MSGS_MESSAGE_IMPLEMENTATION_H
#define CONCERT_MSGS_MESSAGE_IMPLEMENTATION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <concert_msgs/LinkGraph.h>

namespace concert_msgs
{
template <class ContainerAllocator>
struct Implementation_
{
  typedef Implementation_<ContainerAllocator> Type;

  Implementation_()
    : name()
    , link_graph()
    , dot_graph()  {
    }
  Implementation_(const ContainerAllocator& _alloc)
    : name(_alloc)
    , link_graph(_alloc)
    , dot_graph(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  _name_type name;

   typedef  ::concert_msgs::LinkGraph_<ContainerAllocator>  _link_graph_type;
  _link_graph_type link_graph;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _dot_graph_type;
  _dot_graph_type dot_graph;





  typedef boost::shared_ptr< ::concert_msgs::Implementation_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::concert_msgs::Implementation_<ContainerAllocator> const> ConstPtr;

}; // struct Implementation_

typedef ::concert_msgs::Implementation_<std::allocator<void> > Implementation;

typedef boost::shared_ptr< ::concert_msgs::Implementation > ImplementationPtr;
typedef boost::shared_ptr< ::concert_msgs::Implementation const> ImplementationConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::concert_msgs::Implementation_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::concert_msgs::Implementation_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace concert_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'rocon_app_manager_msgs': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/rocon_app_manager_msgs/msg'], 'concert_msgs': ['/mathworks/home/pmurali/Documents/Working/matlab_msg_gen_ros1/glnxa64/src/concert_msgs/msg'], 'std_msgs': ['/mathworks/devel/sbs/31/pmurali.Brobot.j1329247.2/matlab/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::concert_msgs::Implementation_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::concert_msgs::Implementation_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::concert_msgs::Implementation_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::concert_msgs::Implementation_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::concert_msgs::Implementation_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::concert_msgs::Implementation_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::concert_msgs::Implementation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "aeb0655c516d030025a8fe13f0998166";
  }

  static const char* value(const ::concert_msgs::Implementation_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xaeb0655c516d0300ULL;
  static const uint64_t static_value2 = 0x25a8fe13f0998166ULL;
};

template<class ContainerAllocator>
struct DataType< ::concert_msgs::Implementation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "concert_msgs/Implementation";
  }

  static const char* value(const ::concert_msgs::Implementation_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::concert_msgs::Implementation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string name\n"
"LinkGraph link_graph\n"
"string dot_graph\n"
"\n"
"================================================================================\n"
"MSG: concert_msgs/LinkGraph\n"
"# A list of 'named' nodes, e.g. linux.ros.robosem.rocon_teleop\n"
"\n"
"LinkNode[] nodes\n"
"LinkConnection[] topics\n"
"LinkConnection[] actions\n"
"LinkEdge[] edges\n"
"\n"
"================================================================================\n"
"MSG: concert_msgs/LinkNode\n"
"# Representation of a concert client node in the implementation graph\n"
"\n"
"# unique identifier\n"
"string id\n"
"\n"
"# tuple representing the client node - platform.system.robot.app, e.g. linux.ros.turtlebot.turtle_stroll\n"
"string tuple\n"
"\n"
"# Constraints on how many of these nodes may exist\n"
"#   min, max not set -> min = 1, max = 1\n"
"#   min not set -> min = 1\n"
"#   max not set -> max = UNLIMITED_RESOURCE\n"
"# Zero is a valid minimum value\n"
"int8 UNLIMITED_RESOURCE=-1\n"
"int8 min\n"
"int8 max\n"
"\n"
"# Force matching of robot name (only really useful for demos)\n"
"bool force_name_matching\n"
"================================================================================\n"
"MSG: concert_msgs/LinkConnection\n"
"# Representation of a topic/action node in the \n"
"# implementation graph\n"
"\n"
"# unique identifier\n"
"string id\n"
"# type of the topic, e.g. std_msgs/String\n"
"string type\n"
"\n"
"================================================================================\n"
"MSG: concert_msgs/LinkEdge\n"
"# Edge of the implementation graph. Client-Topic, or \n"
"# Topic-Client or Client-Action or Action-Client\n"
"\n"
"string start\n"
"string finish\n"
"string remap_from\n"
"string remap_to\n"
;
  }

  static const char* value(const ::concert_msgs::Implementation_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::concert_msgs::Implementation_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.name);
      stream.next(m.link_graph);
      stream.next(m.dot_graph);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Implementation_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::concert_msgs::Implementation_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::concert_msgs::Implementation_<ContainerAllocator>& v)
  {
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
    s << indent << "link_graph: ";
    s << std::endl;
    Printer< ::concert_msgs::LinkGraph_<ContainerAllocator> >::stream(s, indent + "  ", v.link_graph);
    s << indent << "dot_graph: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.dot_graph);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CONCERT_MSGS_MESSAGE_IMPLEMENTATION_H
