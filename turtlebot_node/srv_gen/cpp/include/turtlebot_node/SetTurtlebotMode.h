/* Auto-generated by genmsg_cpp for file /tmp/buildd/ros-fuerte-turtlebot-1.0.2/debian/ros-fuerte-turtlebot/opt/ros/fuerte/stacks/turtlebot/turtlebot_node/srv/SetTurtlebotMode.srv */
#ifndef TURTLEBOT_NODE_SERVICE_SETTURTLEBOTMODE_H
#define TURTLEBOT_NODE_SERVICE_SETTURTLEBOTMODE_H
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

#include "ros/service_traits.h"




namespace turtlebot_node
{
template <class ContainerAllocator>
struct SetTurtlebotModeRequest_ {
  typedef SetTurtlebotModeRequest_<ContainerAllocator> Type;

  SetTurtlebotModeRequest_()
  : mode(0)
  {
  }

  SetTurtlebotModeRequest_(const ContainerAllocator& _alloc)
  : mode(0)
  {
  }

  typedef uint8_t _mode_type;
  uint8_t mode;


  typedef boost::shared_ptr< ::turtlebot_node::SetTurtlebotModeRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::turtlebot_node::SetTurtlebotModeRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SetTurtlebotModeRequest
typedef  ::turtlebot_node::SetTurtlebotModeRequest_<std::allocator<void> > SetTurtlebotModeRequest;

typedef boost::shared_ptr< ::turtlebot_node::SetTurtlebotModeRequest> SetTurtlebotModeRequestPtr;
typedef boost::shared_ptr< ::turtlebot_node::SetTurtlebotModeRequest const> SetTurtlebotModeRequestConstPtr;


template <class ContainerAllocator>
struct SetTurtlebotModeResponse_ {
  typedef SetTurtlebotModeResponse_<ContainerAllocator> Type;

  SetTurtlebotModeResponse_()
  : valid_mode(false)
  {
  }

  SetTurtlebotModeResponse_(const ContainerAllocator& _alloc)
  : valid_mode(false)
  {
  }

  typedef uint8_t _valid_mode_type;
  uint8_t valid_mode;


  typedef boost::shared_ptr< ::turtlebot_node::SetTurtlebotModeResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::turtlebot_node::SetTurtlebotModeResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SetTurtlebotModeResponse
typedef  ::turtlebot_node::SetTurtlebotModeResponse_<std::allocator<void> > SetTurtlebotModeResponse;

typedef boost::shared_ptr< ::turtlebot_node::SetTurtlebotModeResponse> SetTurtlebotModeResponsePtr;
typedef boost::shared_ptr< ::turtlebot_node::SetTurtlebotModeResponse const> SetTurtlebotModeResponseConstPtr;

struct SetTurtlebotMode
{

typedef SetTurtlebotModeRequest Request;
typedef SetTurtlebotModeResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct SetTurtlebotMode
} // namespace turtlebot_node

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::turtlebot_node::SetTurtlebotModeRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::turtlebot_node::SetTurtlebotModeRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::turtlebot_node::SetTurtlebotModeRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "89b81386720be1cd0ce7a3953fcd1b19";
  }

  static const char* value(const  ::turtlebot_node::SetTurtlebotModeRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x89b81386720be1cdULL;
  static const uint64_t static_value2 = 0x0ce7a3953fcd1b19ULL;
};

template<class ContainerAllocator>
struct DataType< ::turtlebot_node::SetTurtlebotModeRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "turtlebot_node/SetTurtlebotModeRequest";
  }

  static const char* value(const  ::turtlebot_node::SetTurtlebotModeRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::turtlebot_node::SetTurtlebotModeRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "uint8 mode\n\
\n\
";
  }

  static const char* value(const  ::turtlebot_node::SetTurtlebotModeRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::turtlebot_node::SetTurtlebotModeRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::turtlebot_node::SetTurtlebotModeResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::turtlebot_node::SetTurtlebotModeResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::turtlebot_node::SetTurtlebotModeResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ef9db56bf4a60ce42049595d58c32b54";
  }

  static const char* value(const  ::turtlebot_node::SetTurtlebotModeResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xef9db56bf4a60ce4ULL;
  static const uint64_t static_value2 = 0x2049595d58c32b54ULL;
};

template<class ContainerAllocator>
struct DataType< ::turtlebot_node::SetTurtlebotModeResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "turtlebot_node/SetTurtlebotModeResponse";
  }

  static const char* value(const  ::turtlebot_node::SetTurtlebotModeResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::turtlebot_node::SetTurtlebotModeResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool valid_mode\n\
\n\
";
  }

  static const char* value(const  ::turtlebot_node::SetTurtlebotModeResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::turtlebot_node::SetTurtlebotModeResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::turtlebot_node::SetTurtlebotModeRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.mode);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SetTurtlebotModeRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::turtlebot_node::SetTurtlebotModeResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.valid_mode);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SetTurtlebotModeResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<turtlebot_node::SetTurtlebotMode> {
  static const char* value() 
  {
    return "652c4fe00e931153f82f8af90f1da441";
  }

  static const char* value(const turtlebot_node::SetTurtlebotMode&) { return value(); } 
};

template<>
struct DataType<turtlebot_node::SetTurtlebotMode> {
  static const char* value() 
  {
    return "turtlebot_node/SetTurtlebotMode";
  }

  static const char* value(const turtlebot_node::SetTurtlebotMode&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<turtlebot_node::SetTurtlebotModeRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "652c4fe00e931153f82f8af90f1da441";
  }

  static const char* value(const turtlebot_node::SetTurtlebotModeRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<turtlebot_node::SetTurtlebotModeRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "turtlebot_node/SetTurtlebotMode";
  }

  static const char* value(const turtlebot_node::SetTurtlebotModeRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<turtlebot_node::SetTurtlebotModeResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "652c4fe00e931153f82f8af90f1da441";
  }

  static const char* value(const turtlebot_node::SetTurtlebotModeResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<turtlebot_node::SetTurtlebotModeResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "turtlebot_node/SetTurtlebotMode";
  }

  static const char* value(const turtlebot_node::SetTurtlebotModeResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // TURTLEBOT_NODE_SERVICE_SETTURTLEBOTMODE_H

