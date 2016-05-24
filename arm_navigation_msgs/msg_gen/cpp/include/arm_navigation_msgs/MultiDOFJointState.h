/* Auto-generated by genmsg_cpp for file /home/osboxes/groovy_ws/sandbox/arm_navigation_msgs/arm_navigation_msgs/msg/MultiDOFJointState.msg */
#ifndef ARM_NAVIGATION_MSGS_MESSAGE_MULTIDOFJOINTSTATE_H
#define ARM_NAVIGATION_MSGS_MESSAGE_MULTIDOFJOINTSTATE_H
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

#include "geometry_msgs/Pose.h"

namespace arm_navigation_msgs
{
template <class ContainerAllocator>
struct MultiDOFJointState_ {
  typedef MultiDOFJointState_<ContainerAllocator> Type;

  MultiDOFJointState_()
  : stamp()
  , joint_names()
  , frame_ids()
  , child_frame_ids()
  , poses()
  {
  }

  MultiDOFJointState_(const ContainerAllocator& _alloc)
  : stamp()
  , joint_names(_alloc)
  , frame_ids(_alloc)
  , child_frame_ids(_alloc)
  , poses(_alloc)
  {
  }

  typedef ros::Time _stamp_type;
  ros::Time stamp;

  typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _joint_names_type;
  std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  joint_names;

  typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _frame_ids_type;
  std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  frame_ids;

  typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _child_frame_ids_type;
  std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  child_frame_ids;

  typedef std::vector< ::geometry_msgs::Pose_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Pose_<ContainerAllocator> >::other >  _poses_type;
  std::vector< ::geometry_msgs::Pose_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Pose_<ContainerAllocator> >::other >  poses;


  typedef boost::shared_ptr< ::arm_navigation_msgs::MultiDOFJointState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::arm_navigation_msgs::MultiDOFJointState_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct MultiDOFJointState
typedef  ::arm_navigation_msgs::MultiDOFJointState_<std::allocator<void> > MultiDOFJointState;

typedef boost::shared_ptr< ::arm_navigation_msgs::MultiDOFJointState> MultiDOFJointStatePtr;
typedef boost::shared_ptr< ::arm_navigation_msgs::MultiDOFJointState const> MultiDOFJointStateConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::arm_navigation_msgs::MultiDOFJointState_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::arm_navigation_msgs::MultiDOFJointState_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace arm_navigation_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::arm_navigation_msgs::MultiDOFJointState_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::arm_navigation_msgs::MultiDOFJointState_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::arm_navigation_msgs::MultiDOFJointState_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ddd04f13c06870db031db8d5c0a6379d";
  }

  static const char* value(const  ::arm_navigation_msgs::MultiDOFJointState_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xddd04f13c06870dbULL;
  static const uint64_t static_value2 = 0x031db8d5c0a6379dULL;
};

template<class ContainerAllocator>
struct DataType< ::arm_navigation_msgs::MultiDOFJointState_<ContainerAllocator> > {
  static const char* value() 
  {
    return "arm_navigation_msgs/MultiDOFJointState";
  }

  static const char* value(const  ::arm_navigation_msgs::MultiDOFJointState_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::arm_navigation_msgs::MultiDOFJointState_<ContainerAllocator> > {
  static const char* value() 
  {
    return "#A representation of a multi-dof joint state\n\
time stamp\n\
string[] joint_names\n\
string[] frame_ids\n\
string[] child_frame_ids\n\
geometry_msgs/Pose[] poses\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
";
  }

  static const char* value(const  ::arm_navigation_msgs::MultiDOFJointState_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::arm_navigation_msgs::MultiDOFJointState_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.stamp);
    stream.next(m.joint_names);
    stream.next(m.frame_ids);
    stream.next(m.child_frame_ids);
    stream.next(m.poses);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct MultiDOFJointState_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::arm_navigation_msgs::MultiDOFJointState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::arm_navigation_msgs::MultiDOFJointState_<ContainerAllocator> & v) 
  {
    s << indent << "stamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.stamp);
    s << indent << "joint_names[]" << std::endl;
    for (size_t i = 0; i < v.joint_names.size(); ++i)
    {
      s << indent << "  joint_names[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.joint_names[i]);
    }
    s << indent << "frame_ids[]" << std::endl;
    for (size_t i = 0; i < v.frame_ids.size(); ++i)
    {
      s << indent << "  frame_ids[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.frame_ids[i]);
    }
    s << indent << "child_frame_ids[]" << std::endl;
    for (size_t i = 0; i < v.child_frame_ids.size(); ++i)
    {
      s << indent << "  child_frame_ids[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.child_frame_ids[i]);
    }
    s << indent << "poses[]" << std::endl;
    for (size_t i = 0; i < v.poses.size(); ++i)
    {
      s << indent << "  poses[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "    ", v.poses[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // ARM_NAVIGATION_MSGS_MESSAGE_MULTIDOFJOINTSTATE_H

