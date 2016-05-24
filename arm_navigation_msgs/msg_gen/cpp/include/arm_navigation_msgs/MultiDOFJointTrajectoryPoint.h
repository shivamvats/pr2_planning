/* Auto-generated by genmsg_cpp for file /home/osboxes/groovy_ws/sandbox/arm_navigation_msgs/arm_navigation_msgs/msg/MultiDOFJointTrajectoryPoint.msg */
#ifndef ARM_NAVIGATION_MSGS_MESSAGE_MULTIDOFJOINTTRAJECTORYPOINT_H
#define ARM_NAVIGATION_MSGS_MESSAGE_MULTIDOFJOINTTRAJECTORYPOINT_H
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
struct MultiDOFJointTrajectoryPoint_ {
  typedef MultiDOFJointTrajectoryPoint_<ContainerAllocator> Type;

  MultiDOFJointTrajectoryPoint_()
  : poses()
  , time_from_start()
  {
  }

  MultiDOFJointTrajectoryPoint_(const ContainerAllocator& _alloc)
  : poses(_alloc)
  , time_from_start()
  {
  }

  typedef std::vector< ::geometry_msgs::Pose_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Pose_<ContainerAllocator> >::other >  _poses_type;
  std::vector< ::geometry_msgs::Pose_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Pose_<ContainerAllocator> >::other >  poses;

  typedef ros::Duration _time_from_start_type;
  ros::Duration time_from_start;


  typedef boost::shared_ptr< ::arm_navigation_msgs::MultiDOFJointTrajectoryPoint_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::arm_navigation_msgs::MultiDOFJointTrajectoryPoint_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct MultiDOFJointTrajectoryPoint
typedef  ::arm_navigation_msgs::MultiDOFJointTrajectoryPoint_<std::allocator<void> > MultiDOFJointTrajectoryPoint;

typedef boost::shared_ptr< ::arm_navigation_msgs::MultiDOFJointTrajectoryPoint> MultiDOFJointTrajectoryPointPtr;
typedef boost::shared_ptr< ::arm_navigation_msgs::MultiDOFJointTrajectoryPoint const> MultiDOFJointTrajectoryPointConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::arm_navigation_msgs::MultiDOFJointTrajectoryPoint_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::arm_navigation_msgs::MultiDOFJointTrajectoryPoint_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace arm_navigation_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::arm_navigation_msgs::MultiDOFJointTrajectoryPoint_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::arm_navigation_msgs::MultiDOFJointTrajectoryPoint_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::arm_navigation_msgs::MultiDOFJointTrajectoryPoint_<ContainerAllocator> > {
  static const char* value() 
  {
    return "9be3ee3b5fa289b5394ab4ca9e54fa4e";
  }

  static const char* value(const  ::arm_navigation_msgs::MultiDOFJointTrajectoryPoint_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x9be3ee3b5fa289b5ULL;
  static const uint64_t static_value2 = 0x394ab4ca9e54fa4eULL;
};

template<class ContainerAllocator>
struct DataType< ::arm_navigation_msgs::MultiDOFJointTrajectoryPoint_<ContainerAllocator> > {
  static const char* value() 
  {
    return "arm_navigation_msgs/MultiDOFJointTrajectoryPoint";
  }

  static const char* value(const  ::arm_navigation_msgs::MultiDOFJointTrajectoryPoint_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::arm_navigation_msgs::MultiDOFJointTrajectoryPoint_<ContainerAllocator> > {
  static const char* value() 
  {
    return "geometry_msgs/Pose[] poses\n\
duration time_from_start\n\
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

  static const char* value(const  ::arm_navigation_msgs::MultiDOFJointTrajectoryPoint_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::arm_navigation_msgs::MultiDOFJointTrajectoryPoint_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.poses);
    stream.next(m.time_from_start);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct MultiDOFJointTrajectoryPoint_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::arm_navigation_msgs::MultiDOFJointTrajectoryPoint_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::arm_navigation_msgs::MultiDOFJointTrajectoryPoint_<ContainerAllocator> & v) 
  {
    s << indent << "poses[]" << std::endl;
    for (size_t i = 0; i < v.poses.size(); ++i)
    {
      s << indent << "  poses[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "    ", v.poses[i]);
    }
    s << indent << "time_from_start: ";
    Printer<ros::Duration>::stream(s, indent + "  ", v.time_from_start);
  }
};


} // namespace message_operations
} // namespace ros

#endif // ARM_NAVIGATION_MSGS_MESSAGE_MULTIDOFJOINTTRAJECTORYPOINT_H

