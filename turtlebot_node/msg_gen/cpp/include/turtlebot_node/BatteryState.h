/* Auto-generated by genmsg_cpp for file /tmp/buildd/ros-fuerte-turtlebot-1.0.2/debian/ros-fuerte-turtlebot/opt/ros/fuerte/stacks/turtlebot/turtlebot_node/msg/BatteryState.msg */
#ifndef TURTLEBOT_NODE_MESSAGE_BATTERYSTATE_H
#define TURTLEBOT_NODE_MESSAGE_BATTERYSTATE_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "std_msgs/Header.h"

namespace turtlebot_node
{
template <class ContainerAllocator>
struct BatteryState_ {
  typedef BatteryState_<ContainerAllocator> Type;

  BatteryState_()
  : header()
  , temperature(0)
  , charge(0)
  , capacity(0)
  {
  }

  BatteryState_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , temperature(0)
  , charge(0)
  , capacity(0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef int8_t _temperature_type;
  int8_t temperature;

  typedef uint16_t _charge_type;
  uint16_t charge;

  typedef uint16_t _capacity_type;
  uint16_t capacity;


  typedef boost::shared_ptr< ::turtlebot_node::BatteryState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::turtlebot_node::BatteryState_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct BatteryState
typedef  ::turtlebot_node::BatteryState_<std::allocator<void> > BatteryState;

typedef boost::shared_ptr< ::turtlebot_node::BatteryState> BatteryStatePtr;
typedef boost::shared_ptr< ::turtlebot_node::BatteryState const> BatteryStateConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::turtlebot_node::BatteryState_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::turtlebot_node::BatteryState_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace turtlebot_node

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::turtlebot_node::BatteryState_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::turtlebot_node::BatteryState_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::turtlebot_node::BatteryState_<ContainerAllocator> > {
  static const char* value() 
  {
    return "481447a4e24f212e7b403e4f04aa2ac9";
  }

  static const char* value(const  ::turtlebot_node::BatteryState_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x481447a4e24f212eULL;
  static const uint64_t static_value2 = 0x7b403e4f04aa2ac9ULL;
};

template<class ContainerAllocator>
struct DataType< ::turtlebot_node::BatteryState_<ContainerAllocator> > {
  static const char* value() 
  {
    return "turtlebot_node/BatteryState";
  }

  static const char* value(const  ::turtlebot_node::BatteryState_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::turtlebot_node::BatteryState_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
int8 temperature\n\
uint16 charge\n\
uint16 capacity\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::turtlebot_node::BatteryState_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::turtlebot_node::BatteryState_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::turtlebot_node::BatteryState_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::turtlebot_node::BatteryState_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.temperature);
    stream.next(m.charge);
    stream.next(m.capacity);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct BatteryState_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::turtlebot_node::BatteryState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::turtlebot_node::BatteryState_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "temperature: ";
    Printer<int8_t>::stream(s, indent + "  ", v.temperature);
    s << indent << "charge: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.charge);
    s << indent << "capacity: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.capacity);
  }
};


} // namespace message_operations
} // namespace ros

#endif // TURTLEBOT_NODE_MESSAGE_BATTERYSTATE_H
