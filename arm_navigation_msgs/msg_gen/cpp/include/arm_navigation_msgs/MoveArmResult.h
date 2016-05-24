/* Auto-generated by genmsg_cpp for file /home/osboxes/groovy_ws/sandbox/arm_navigation_msgs/arm_navigation_msgs/msg/MoveArmResult.msg */
#ifndef ARM_NAVIGATION_MSGS_MESSAGE_MOVEARMRESULT_H
#define ARM_NAVIGATION_MSGS_MESSAGE_MOVEARMRESULT_H
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

#include "arm_navigation_msgs/ArmNavigationErrorCodes.h"
#include "arm_navigation_msgs/ContactInformation.h"

namespace arm_navigation_msgs
{
template <class ContainerAllocator>
struct MoveArmResult_ {
  typedef MoveArmResult_<ContainerAllocator> Type;

  MoveArmResult_()
  : error_code()
  , contacts()
  {
  }

  MoveArmResult_(const ContainerAllocator& _alloc)
  : error_code(_alloc)
  , contacts(_alloc)
  {
  }

  typedef  ::arm_navigation_msgs::ArmNavigationErrorCodes_<ContainerAllocator>  _error_code_type;
   ::arm_navigation_msgs::ArmNavigationErrorCodes_<ContainerAllocator>  error_code;

  typedef std::vector< ::arm_navigation_msgs::ContactInformation_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::arm_navigation_msgs::ContactInformation_<ContainerAllocator> >::other >  _contacts_type;
  std::vector< ::arm_navigation_msgs::ContactInformation_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::arm_navigation_msgs::ContactInformation_<ContainerAllocator> >::other >  contacts;


  typedef boost::shared_ptr< ::arm_navigation_msgs::MoveArmResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::arm_navigation_msgs::MoveArmResult_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct MoveArmResult
typedef  ::arm_navigation_msgs::MoveArmResult_<std::allocator<void> > MoveArmResult;

typedef boost::shared_ptr< ::arm_navigation_msgs::MoveArmResult> MoveArmResultPtr;
typedef boost::shared_ptr< ::arm_navigation_msgs::MoveArmResult const> MoveArmResultConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::arm_navigation_msgs::MoveArmResult_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::arm_navigation_msgs::MoveArmResult_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace arm_navigation_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::arm_navigation_msgs::MoveArmResult_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::arm_navigation_msgs::MoveArmResult_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::arm_navigation_msgs::MoveArmResult_<ContainerAllocator> > {
  static const char* value() 
  {
    return "3229301226a0605e3ffc9dfdaeac662f";
  }

  static const char* value(const  ::arm_navigation_msgs::MoveArmResult_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x3229301226a0605eULL;
  static const uint64_t static_value2 = 0x3ffc9dfdaeac662fULL;
};

template<class ContainerAllocator>
struct DataType< ::arm_navigation_msgs::MoveArmResult_<ContainerAllocator> > {
  static const char* value() 
  {
    return "arm_navigation_msgs/MoveArmResult";
  }

  static const char* value(const  ::arm_navigation_msgs::MoveArmResult_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::arm_navigation_msgs::MoveArmResult_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# An error code reflecting what went wrong\n\
ArmNavigationErrorCodes error_code\n\
\n\
ContactInformation[] contacts\n\
\n\
================================================================================\n\
MSG: arm_navigation_msgs/ArmNavigationErrorCodes\n\
int32 val\n\
\n\
# overall behavior\n\
int32 PLANNING_FAILED=-1\n\
int32 SUCCESS=1\n\
int32 TIMED_OUT=-2\n\
\n\
# start state errors\n\
int32 START_STATE_IN_COLLISION=-3\n\
int32 START_STATE_VIOLATES_PATH_CONSTRAINTS=-4\n\
\n\
# goal errors\n\
int32 GOAL_IN_COLLISION=-5\n\
int32 GOAL_VIOLATES_PATH_CONSTRAINTS=-6\n\
\n\
# robot state\n\
int32 INVALID_ROBOT_STATE=-7\n\
int32 INCOMPLETE_ROBOT_STATE=-8\n\
\n\
# planning request errors\n\
int32 INVALID_PLANNER_ID=-9\n\
int32 INVALID_NUM_PLANNING_ATTEMPTS=-10\n\
int32 INVALID_ALLOWED_PLANNING_TIME=-11\n\
int32 INVALID_GROUP_NAME=-12\n\
int32 INVALID_GOAL_JOINT_CONSTRAINTS=-13\n\
int32 INVALID_GOAL_POSITION_CONSTRAINTS=-14\n\
int32 INVALID_GOAL_ORIENTATION_CONSTRAINTS=-15\n\
int32 INVALID_PATH_JOINT_CONSTRAINTS=-16\n\
int32 INVALID_PATH_POSITION_CONSTRAINTS=-17\n\
int32 INVALID_PATH_ORIENTATION_CONSTRAINTS=-18\n\
\n\
# state/trajectory monitor errors\n\
int32 INVALID_TRAJECTORY=-19\n\
int32 INVALID_INDEX=-20\n\
int32 JOINT_LIMITS_VIOLATED=-21\n\
int32 PATH_CONSTRAINTS_VIOLATED=-22\n\
int32 COLLISION_CONSTRAINTS_VIOLATED=-23\n\
int32 GOAL_CONSTRAINTS_VIOLATED=-24\n\
int32 JOINTS_NOT_MOVING=-25\n\
int32 TRAJECTORY_CONTROLLER_FAILED=-26\n\
\n\
# system errors\n\
int32 FRAME_TRANSFORM_FAILURE=-27\n\
int32 COLLISION_CHECKING_UNAVAILABLE=-28\n\
int32 ROBOT_STATE_STALE=-29\n\
int32 SENSOR_INFO_STALE=-30\n\
\n\
# kinematics errors\n\
int32 NO_IK_SOLUTION=-31\n\
int32 INVALID_LINK_NAME=-32\n\
int32 IK_LINK_IN_COLLISION=-33\n\
int32 NO_FK_SOLUTION=-34\n\
int32 KINEMATICS_STATE_IN_COLLISION=-35\n\
\n\
# general errors\n\
int32 INVALID_TIMEOUT=-36\n\
\n\
\n\
================================================================================\n\
MSG: arm_navigation_msgs/ContactInformation\n\
# Standard ROS header contains information \n\
# about the frame in which this \n\
# contact is specified\n\
Header header\n\
\n\
# Position of the contact point\n\
geometry_msgs/Point position\n\
\n\
# Normal corresponding to the contact point\n\
geometry_msgs/Vector3 normal \n\
\n\
# Depth of contact point\n\
float64 depth\n\
\n\
# Name of the first body that is in contact\n\
# This could be a link or a namespace that represents a body\n\
string contact_body_1\n\
string attached_body_1\n\
uint32 body_type_1\n\
\n\
# Name of the second body that is in contact\n\
# This could be a link or a namespace that represents a body\n\
string contact_body_2\n\
string attached_body_2\n\
uint32 body_type_2\n\
\n\
uint32 ROBOT_LINK=0\n\
uint32 OBJECT=1\n\
uint32 ATTACHED_BODY=2\n\
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
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const  ::arm_navigation_msgs::MoveArmResult_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::arm_navigation_msgs::MoveArmResult_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.error_code);
    stream.next(m.contacts);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct MoveArmResult_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::arm_navigation_msgs::MoveArmResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::arm_navigation_msgs::MoveArmResult_<ContainerAllocator> & v) 
  {
    s << indent << "error_code: ";
s << std::endl;
    Printer< ::arm_navigation_msgs::ArmNavigationErrorCodes_<ContainerAllocator> >::stream(s, indent + "  ", v.error_code);
    s << indent << "contacts[]" << std::endl;
    for (size_t i = 0; i < v.contacts.size(); ++i)
    {
      s << indent << "  contacts[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::arm_navigation_msgs::ContactInformation_<ContainerAllocator> >::stream(s, indent + "    ", v.contacts[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // ARM_NAVIGATION_MSGS_MESSAGE_MOVEARMRESULT_H

