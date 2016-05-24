/* Auto-generated by genmsg_cpp for file /home/osboxes/groovy_ws/sandbox/arm_navigation_msgs/arm_navigation_msgs/msg/RobotTrajectory.msg */
#ifndef ARM_NAVIGATION_MSGS_MESSAGE_ROBOTTRAJECTORY_H
#define ARM_NAVIGATION_MSGS_MESSAGE_ROBOTTRAJECTORY_H
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

#include "trajectory_msgs/JointTrajectory.h"
#include "arm_navigation_msgs/MultiDOFJointTrajectory.h"

namespace arm_navigation_msgs
{
template <class ContainerAllocator>
struct RobotTrajectory_ {
  typedef RobotTrajectory_<ContainerAllocator> Type;

  RobotTrajectory_()
  : joint_trajectory()
  , multi_dof_joint_trajectory()
  {
  }

  RobotTrajectory_(const ContainerAllocator& _alloc)
  : joint_trajectory(_alloc)
  , multi_dof_joint_trajectory(_alloc)
  {
  }

  typedef  ::trajectory_msgs::JointTrajectory_<ContainerAllocator>  _joint_trajectory_type;
   ::trajectory_msgs::JointTrajectory_<ContainerAllocator>  joint_trajectory;

  typedef  ::arm_navigation_msgs::MultiDOFJointTrajectory_<ContainerAllocator>  _multi_dof_joint_trajectory_type;
   ::arm_navigation_msgs::MultiDOFJointTrajectory_<ContainerAllocator>  multi_dof_joint_trajectory;


  typedef boost::shared_ptr< ::arm_navigation_msgs::RobotTrajectory_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::arm_navigation_msgs::RobotTrajectory_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct RobotTrajectory
typedef  ::arm_navigation_msgs::RobotTrajectory_<std::allocator<void> > RobotTrajectory;

typedef boost::shared_ptr< ::arm_navigation_msgs::RobotTrajectory> RobotTrajectoryPtr;
typedef boost::shared_ptr< ::arm_navigation_msgs::RobotTrajectory const> RobotTrajectoryConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::arm_navigation_msgs::RobotTrajectory_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::arm_navigation_msgs::RobotTrajectory_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace arm_navigation_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::arm_navigation_msgs::RobotTrajectory_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::arm_navigation_msgs::RobotTrajectory_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::arm_navigation_msgs::RobotTrajectory_<ContainerAllocator> > {
  static const char* value() 
  {
    return "5bc8324620001e5c07a09d0bbfaaf093";
  }

  static const char* value(const  ::arm_navigation_msgs::RobotTrajectory_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x5bc8324620001e5cULL;
  static const uint64_t static_value2 = 0x07a09d0bbfaaf093ULL;
};

template<class ContainerAllocator>
struct DataType< ::arm_navigation_msgs::RobotTrajectory_<ContainerAllocator> > {
  static const char* value() 
  {
    return "arm_navigation_msgs/RobotTrajectory";
  }

  static const char* value(const  ::arm_navigation_msgs::RobotTrajectory_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::arm_navigation_msgs::RobotTrajectory_<ContainerAllocator> > {
  static const char* value() 
  {
    return "trajectory_msgs/JointTrajectory joint_trajectory\n\
arm_navigation_msgs/MultiDOFJointTrajectory multi_dof_joint_trajectory\n\
\n\
================================================================================\n\
MSG: trajectory_msgs/JointTrajectory\n\
Header header\n\
string[] joint_names\n\
JointTrajectoryPoint[] points\n\
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
================================================================================\n\
MSG: trajectory_msgs/JointTrajectoryPoint\n\
float64[] positions\n\
float64[] velocities\n\
float64[] accelerations\n\
duration time_from_start\n\
================================================================================\n\
MSG: arm_navigation_msgs/MultiDOFJointTrajectory\n\
#A representation of a multi-dof joint trajectory\n\
duration stamp\n\
string[] joint_names\n\
string[] frame_ids\n\
string[] child_frame_ids\n\
MultiDOFJointTrajectoryPoint[] points\n\
\n\
================================================================================\n\
MSG: arm_navigation_msgs/MultiDOFJointTrajectoryPoint\n\
geometry_msgs/Pose[] poses\n\
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

  static const char* value(const  ::arm_navigation_msgs::RobotTrajectory_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::arm_navigation_msgs::RobotTrajectory_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.joint_trajectory);
    stream.next(m.multi_dof_joint_trajectory);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct RobotTrajectory_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::arm_navigation_msgs::RobotTrajectory_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::arm_navigation_msgs::RobotTrajectory_<ContainerAllocator> & v) 
  {
    s << indent << "joint_trajectory: ";
s << std::endl;
    Printer< ::trajectory_msgs::JointTrajectory_<ContainerAllocator> >::stream(s, indent + "  ", v.joint_trajectory);
    s << indent << "multi_dof_joint_trajectory: ";
s << std::endl;
    Printer< ::arm_navigation_msgs::MultiDOFJointTrajectory_<ContainerAllocator> >::stream(s, indent + "  ", v.multi_dof_joint_trajectory);
  }
};


} // namespace message_operations
} // namespace ros

#endif // ARM_NAVIGATION_MSGS_MESSAGE_ROBOTTRAJECTORY_H

