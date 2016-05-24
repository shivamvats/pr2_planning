/* Auto-generated by genmsg_cpp for file /home/osboxes/groovy_ws/sandbox/arm_navigation_msgs/arm_navigation_msgs/msg/CollisionObjectOperation.msg */
#ifndef ARM_NAVIGATION_MSGS_MESSAGE_COLLISIONOBJECTOPERATION_H
#define ARM_NAVIGATION_MSGS_MESSAGE_COLLISIONOBJECTOPERATION_H
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


namespace arm_navigation_msgs
{
template <class ContainerAllocator>
struct CollisionObjectOperation_ {
  typedef CollisionObjectOperation_<ContainerAllocator> Type;

  CollisionObjectOperation_()
  : operation(0)
  {
  }

  CollisionObjectOperation_(const ContainerAllocator& _alloc)
  : operation(0)
  {
  }

  typedef int8_t _operation_type;
  int8_t operation;

  enum { ADD = 0 };
  enum { REMOVE = 1 };
  enum { DETACH_AND_ADD_AS_OBJECT = 2 };
  enum { ATTACH_AND_REMOVE_AS_OBJECT = 3 };

  typedef boost::shared_ptr< ::arm_navigation_msgs::CollisionObjectOperation_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::arm_navigation_msgs::CollisionObjectOperation_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct CollisionObjectOperation
typedef  ::arm_navigation_msgs::CollisionObjectOperation_<std::allocator<void> > CollisionObjectOperation;

typedef boost::shared_ptr< ::arm_navigation_msgs::CollisionObjectOperation> CollisionObjectOperationPtr;
typedef boost::shared_ptr< ::arm_navigation_msgs::CollisionObjectOperation const> CollisionObjectOperationConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::arm_navigation_msgs::CollisionObjectOperation_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::arm_navigation_msgs::CollisionObjectOperation_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace arm_navigation_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::arm_navigation_msgs::CollisionObjectOperation_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::arm_navigation_msgs::CollisionObjectOperation_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::arm_navigation_msgs::CollisionObjectOperation_<ContainerAllocator> > {
  static const char* value() 
  {
    return "66a2b3b971d193145f8da8c3e401a474";
  }

  static const char* value(const  ::arm_navigation_msgs::CollisionObjectOperation_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x66a2b3b971d19314ULL;
  static const uint64_t static_value2 = 0x5f8da8c3e401a474ULL;
};

template<class ContainerAllocator>
struct DataType< ::arm_navigation_msgs::CollisionObjectOperation_<ContainerAllocator> > {
  static const char* value() 
  {
    return "arm_navigation_msgs/CollisionObjectOperation";
  }

  static const char* value(const  ::arm_navigation_msgs::CollisionObjectOperation_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::arm_navigation_msgs::CollisionObjectOperation_<ContainerAllocator> > {
  static const char* value() 
  {
    return "#Puts the object into the environment\n\
#or updates the object if already added\n\
byte ADD=0\n\
\n\
#Removes the object from the environment entirely\n\
byte REMOVE=1\n\
\n\
#Only valid within the context of a CollisionAttachedObject message\n\
#Will be ignored if sent with an CollisionObject message\n\
#Takes an attached object, detaches from the attached link\n\
#But adds back in as regular object\n\
byte DETACH_AND_ADD_AS_OBJECT=2\n\
\n\
#Only valid within the context of a CollisionAttachedObject message\n\
#Will be ignored if sent with an CollisionObject message\n\
#Takes current object in the environment and removes it as\n\
#a regular object\n\
byte ATTACH_AND_REMOVE_AS_OBJECT=3\n\
\n\
# Byte code for operation\n\
byte operation\n\
\n\
";
  }

  static const char* value(const  ::arm_navigation_msgs::CollisionObjectOperation_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::arm_navigation_msgs::CollisionObjectOperation_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::arm_navigation_msgs::CollisionObjectOperation_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.operation);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct CollisionObjectOperation_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::arm_navigation_msgs::CollisionObjectOperation_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::arm_navigation_msgs::CollisionObjectOperation_<ContainerAllocator> & v) 
  {
    s << indent << "operation: ";
    Printer<int8_t>::stream(s, indent + "  ", v.operation);
  }
};


} // namespace message_operations
} // namespace ros

#endif // ARM_NAVIGATION_MSGS_MESSAGE_COLLISIONOBJECTOPERATION_H
