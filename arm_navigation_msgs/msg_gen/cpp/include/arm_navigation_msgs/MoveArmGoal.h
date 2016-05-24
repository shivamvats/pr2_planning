/* Auto-generated by genmsg_cpp for file /home/osboxes/groovy_ws/sandbox/arm_navigation_msgs/arm_navigation_msgs/msg/MoveArmGoal.msg */
#ifndef ARM_NAVIGATION_MSGS_MESSAGE_MOVEARMGOAL_H
#define ARM_NAVIGATION_MSGS_MESSAGE_MOVEARMGOAL_H
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

#include "arm_navigation_msgs/PlanningScene.h"
#include "arm_navigation_msgs/MotionPlanRequest.h"
#include "arm_navigation_msgs/OrderedCollisionOperations.h"

namespace arm_navigation_msgs
{
template <class ContainerAllocator>
struct MoveArmGoal_ {
  typedef MoveArmGoal_<ContainerAllocator> Type;

  MoveArmGoal_()
  : planner_service_name()
  , planning_scene_diff()
  , motion_plan_request()
  , operations()
  , accept_partial_plans(false)
  , accept_invalid_goals(false)
  , disable_ik(false)
  , disable_collision_monitoring(false)
  {
  }

  MoveArmGoal_(const ContainerAllocator& _alloc)
  : planner_service_name(_alloc)
  , planning_scene_diff(_alloc)
  , motion_plan_request(_alloc)
  , operations(_alloc)
  , accept_partial_plans(false)
  , accept_invalid_goals(false)
  , disable_ik(false)
  , disable_collision_monitoring(false)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _planner_service_name_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  planner_service_name;

  typedef  ::arm_navigation_msgs::PlanningScene_<ContainerAllocator>  _planning_scene_diff_type;
   ::arm_navigation_msgs::PlanningScene_<ContainerAllocator>  planning_scene_diff;

  typedef  ::arm_navigation_msgs::MotionPlanRequest_<ContainerAllocator>  _motion_plan_request_type;
   ::arm_navigation_msgs::MotionPlanRequest_<ContainerAllocator>  motion_plan_request;

  typedef  ::arm_navigation_msgs::OrderedCollisionOperations_<ContainerAllocator>  _operations_type;
   ::arm_navigation_msgs::OrderedCollisionOperations_<ContainerAllocator>  operations;

  typedef uint8_t _accept_partial_plans_type;
  uint8_t accept_partial_plans;

  typedef uint8_t _accept_invalid_goals_type;
  uint8_t accept_invalid_goals;

  typedef uint8_t _disable_ik_type;
  uint8_t disable_ik;

  typedef uint8_t _disable_collision_monitoring_type;
  uint8_t disable_collision_monitoring;


  typedef boost::shared_ptr< ::arm_navigation_msgs::MoveArmGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::arm_navigation_msgs::MoveArmGoal_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct MoveArmGoal
typedef  ::arm_navigation_msgs::MoveArmGoal_<std::allocator<void> > MoveArmGoal;

typedef boost::shared_ptr< ::arm_navigation_msgs::MoveArmGoal> MoveArmGoalPtr;
typedef boost::shared_ptr< ::arm_navigation_msgs::MoveArmGoal const> MoveArmGoalConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::arm_navigation_msgs::MoveArmGoal_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::arm_navigation_msgs::MoveArmGoal_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace arm_navigation_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::arm_navigation_msgs::MoveArmGoal_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::arm_navigation_msgs::MoveArmGoal_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::arm_navigation_msgs::MoveArmGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "229373059043ad35d3ceeb2161f005d6";
  }

  static const char* value(const  ::arm_navigation_msgs::MoveArmGoal_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x229373059043ad35ULL;
  static const uint64_t static_value2 = 0xd3ceeb2161f005d6ULL;
};

template<class ContainerAllocator>
struct DataType< ::arm_navigation_msgs::MoveArmGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "arm_navigation_msgs/MoveArmGoal";
  }

  static const char* value(const  ::arm_navigation_msgs::MoveArmGoal_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::arm_navigation_msgs::MoveArmGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# Service name to call for getting a motion plan\n\
# Move arm will call a service on this service name \n\
# using the MotionPlanRequest specified here\n\
string planner_service_name\n\
\n\
# A planning scene diff\n\
PlanningScene planning_scene_diff\n\
\n\
# A motion planning request\n\
MotionPlanRequest motion_plan_request\n\
\n\
# OPTIONAL: Diff uses ordered collision operations in addition to allowed_collision_matrix\n\
arm_navigation_msgs/OrderedCollisionOperations operations\n\
\n\
# OPTIONAL flag\n\
# Setting this flag to true will allow move_arm to accept plans that do not go all the way to the goal\n\
bool accept_partial_plans\n\
\n\
# OPTIONAL flag\n\
# Setting this flag to true will allow move_arm to accept invalid goals\n\
# This is useful if you are using a planner like CHOMP along with a noisy rapidly changing collision map\n\
# and you would like to plan to a goal near an object.\n\
bool accept_invalid_goals\n\
\n\
# OPTIONAL flag\n\
# Setting this flag to true will disable the call to IK for a pose goal\n\
bool disable_ik\n\
\n\
# OPTIONAL flag\n\
# Setting this flag to true will disable collision monitoring during execution of a trajectory\n\
bool disable_collision_monitoring\n\
\n\
================================================================================\n\
MSG: arm_navigation_msgs/PlanningScene\n\
#full robot state\n\
arm_navigation_msgs/RobotState robot_state\n\
\n\
#additional frames for duplicating tf\n\
geometry_msgs/TransformStamped[] fixed_frame_transforms\n\
\n\
#full allowed collision matrix\n\
AllowedCollisionMatrix allowed_collision_matrix\n\
\n\
#allowed contacts\n\
arm_navigation_msgs/AllowedContactSpecification[] allowed_contacts\n\
\n\
#all link paddings\n\
arm_navigation_msgs/LinkPadding[] link_padding\n\
\n\
#collision objects\n\
arm_navigation_msgs/CollisionObject[] collision_objects\n\
arm_navigation_msgs/AttachedCollisionObject[] attached_collision_objects\n\
\n\
#the collision map\n\
arm_navigation_msgs/CollisionMap collision_map\n\
\n\
================================================================================\n\
MSG: arm_navigation_msgs/RobotState\n\
# This message contains information about the robot state, i.e. the positions of its joints and links\n\
sensor_msgs/JointState joint_state\n\
arm_navigation_msgs/MultiDOFJointState multi_dof_joint_state\n\
\n\
================================================================================\n\
MSG: sensor_msgs/JointState\n\
# This is a message that holds data to describe the state of a set of torque controlled joints. \n\
#\n\
# The state of each joint (revolute or prismatic) is defined by:\n\
#  * the position of the joint (rad or m),\n\
#  * the velocity of the joint (rad/s or m/s) and \n\
#  * the effort that is applied in the joint (Nm or N).\n\
#\n\
# Each joint is uniquely identified by its name\n\
# The header specifies the time at which the joint states were recorded. All the joint states\n\
# in one message have to be recorded at the same time.\n\
#\n\
# This message consists of a multiple arrays, one for each part of the joint state. \n\
# The goal is to make each of the fields optional. When e.g. your joints have no\n\
# effort associated with them, you can leave the effort array empty. \n\
#\n\
# All arrays in this message should have the same size, or be empty.\n\
# This is the only way to uniquely associate the joint name with the correct\n\
# states.\n\
\n\
\n\
Header header\n\
\n\
string[] name\n\
float64[] position\n\
float64[] velocity\n\
float64[] effort\n\
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
================================================================================\n\
MSG: arm_navigation_msgs/MultiDOFJointState\n\
#A representation of a multi-dof joint state\n\
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
================================================================================\n\
MSG: geometry_msgs/TransformStamped\n\
# This expresses a transform from coordinate frame header.frame_id\n\
# to the coordinate frame child_frame_id\n\
#\n\
# This message is mostly used by the \n\
# <a href=\"http://www.ros.org/wiki/tf\">tf</a> package. \n\
# See its documentation for more information.\n\
\n\
Header header\n\
string child_frame_id # the frame id of the child frame\n\
Transform transform\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Transform\n\
# This represents the transform between two coordinate frames in free space.\n\
\n\
Vector3 translation\n\
Quaternion rotation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
================================================================================\n\
MSG: arm_navigation_msgs/AllowedCollisionMatrix\n\
# the list of link names in the matrix\n\
string[] link_names\n\
\n\
# the individual entries in the allowed collision matrix\n\
# symmetric, with same order as link_names\n\
AllowedCollisionEntry[] entries\n\
\n\
================================================================================\n\
MSG: arm_navigation_msgs/AllowedCollisionEntry\n\
# whether or not collision checking is enabled\n\
bool[] enabled\n\
\n\
================================================================================\n\
MSG: arm_navigation_msgs/AllowedContactSpecification\n\
# The names of the regions\n\
string name\n\
\n\
# The shape of the region in the environment\n\
arm_navigation_msgs/Shape shape\n\
\n\
# The pose of the space defining the region\n\
geometry_msgs/PoseStamped pose_stamped\n\
\n\
# The set of links that will be allowed to have penetration contact within this region\n\
string[] link_names\n\
\n\
# The maximum penetration depth allowed for every link\n\
float64 penetration_depth\n\
\n\
================================================================================\n\
MSG: arm_navigation_msgs/Shape\n\
byte SPHERE=0\n\
byte BOX=1\n\
byte CYLINDER=2\n\
byte MESH=3\n\
\n\
byte type\n\
\n\
\n\
#### define sphere, box, cylinder ####\n\
# the origin of each shape is considered at the shape's center\n\
\n\
# for sphere\n\
# radius := dimensions[0]\n\
\n\
# for cylinder\n\
# radius := dimensions[0]\n\
# length := dimensions[1]\n\
# the length is along the Z axis\n\
\n\
# for box\n\
# size_x := dimensions[0]\n\
# size_y := dimensions[1]\n\
# size_z := dimensions[2]\n\
float64[] dimensions\n\
\n\
\n\
#### define mesh ####\n\
\n\
# list of triangles; triangle k is defined by tre vertices located\n\
# at indices triangles[3k], triangles[3k+1], triangles[3k+2]\n\
int32[] triangles\n\
geometry_msgs/Point[] vertices\n\
\n\
================================================================================\n\
MSG: geometry_msgs/PoseStamped\n\
# A Pose with reference coordinate frame and timestamp\n\
Header header\n\
Pose pose\n\
\n\
================================================================================\n\
MSG: arm_navigation_msgs/LinkPadding\n\
#name for the link\n\
string link_name\n\
\n\
# padding to apply to the link\n\
float64 padding\n\
\n\
================================================================================\n\
MSG: arm_navigation_msgs/CollisionObject\n\
# a header, used for interpreting the poses\n\
Header header\n\
\n\
# the id of the object\n\
string id\n\
\n\
# The padding used for filtering points near the object.\n\
# This does not affect collision checking for the object.  \n\
# Set to negative to get zero padding.\n\
float32 padding\n\
\n\
#This contains what is to be done with the object\n\
CollisionObjectOperation operation\n\
\n\
#the shapes associated with the object\n\
arm_navigation_msgs/Shape[] shapes\n\
\n\
#the poses associated with the shapes - will be transformed using the header\n\
geometry_msgs/Pose[] poses\n\
\n\
================================================================================\n\
MSG: arm_navigation_msgs/CollisionObjectOperation\n\
#Puts the object into the environment\n\
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
================================================================================\n\
MSG: arm_navigation_msgs/AttachedCollisionObject\n\
# The CollisionObject will be attached with a fixed joint to this link\n\
# If link name is set to REMOVE_ALL_ATTACHED_OBJECTS and object.operation \n\
# is set to REMOVE will remove all attached bodies attached to any object\n\
string link_name\n\
\n\
#Reserved for indicating that all attached objects should be removed\n\
string REMOVE_ALL_ATTACHED_OBJECTS = \"all\"\n\
\n\
#This contains the actual shapes and poses for the CollisionObject\n\
#to be attached to the link\n\
#If action is remove and no object.id is set, all objects\n\
#attached to the link indicated by link_name will be removed\n\
CollisionObject object\n\
\n\
# The set of links that the attached objects are allowed to touch\n\
# by default - the link_name is included by default\n\
string[] touch_links\n\
\n\
================================================================================\n\
MSG: arm_navigation_msgs/CollisionMap\n\
#header for interpreting box positions\n\
Header header\n\
\n\
#boxes for use in collision testing\n\
OrientedBoundingBox[] boxes\n\
\n\
================================================================================\n\
MSG: arm_navigation_msgs/OrientedBoundingBox\n\
#the center of the box\n\
geometry_msgs/Point32 center\n\
\n\
#the extents of the box, assuming the center is at the point\n\
geometry_msgs/Point32 extents\n\
\n\
#the axis of the box\n\
geometry_msgs/Point32 axis\n\
\n\
#the angle of rotation around the axis\n\
float32 angle\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point32\n\
# This contains the position of a point in free space(with 32 bits of precision).\n\
# It is recommeded to use Point wherever possible instead of Point32.  \n\
# \n\
# This recommendation is to promote interoperability.  \n\
#\n\
# This message is designed to take up less space when sending\n\
# lots of points at once, as in the case of a PointCloud.  \n\
\n\
float32 x\n\
float32 y\n\
float32 z\n\
================================================================================\n\
MSG: arm_navigation_msgs/MotionPlanRequest\n\
# This service contains the definition for a request to the motion\n\
# planner and the output it provides\n\
\n\
# Parameters for the workspace that the planner should work inside\n\
arm_navigation_msgs/WorkspaceParameters workspace_parameters\n\
\n\
# Starting state updates. If certain joints should be considered\n\
# at positions other than the current ones, these positions should\n\
# be set here\n\
arm_navigation_msgs/RobotState start_state\n\
\n\
# The goal state for the model to plan for. The goal is achieved\n\
# if all constraints are satisfied\n\
arm_navigation_msgs/Constraints goal_constraints\n\
\n\
# No state at any point along the path in the produced motion plan will violate these constraints\n\
arm_navigation_msgs/Constraints path_constraints\n\
\n\
# The name of the motion planner to use. If no name is specified,\n\
# a default motion planner will be used\n\
string planner_id\n\
\n\
# The name of the group of joints on which this planner is operating\n\
string group_name\n\
\n\
# The number of times this plan is to be computed. Shortest solution\n\
# will be reported.\n\
int32 num_planning_attempts\n\
\n\
# The maximum amount of time the motion planner is allowed to plan for\n\
duration allowed_planning_time\n\
\n\
# An expected path duration (in seconds) along with an expected discretization of the path allows the planner to determine the discretization of the trajectory that it returns\n\
duration expected_path_duration\n\
duration expected_path_dt\n\
\n\
================================================================================\n\
MSG: arm_navigation_msgs/WorkspaceParameters\n\
# This message contains a set of parameters useful in\n\
# setting up the workspace for planning\n\
arm_navigation_msgs/Shape  workspace_region_shape\n\
geometry_msgs/PoseStamped    workspace_region_pose\n\
\n\
\n\
================================================================================\n\
MSG: arm_navigation_msgs/Constraints\n\
# This message contains a list of motion planning constraints.\n\
\n\
arm_navigation_msgs/JointConstraint[] joint_constraints\n\
arm_navigation_msgs/PositionConstraint[] position_constraints\n\
arm_navigation_msgs/OrientationConstraint[] orientation_constraints\n\
arm_navigation_msgs/VisibilityConstraint[] visibility_constraints\n\
\n\
================================================================================\n\
MSG: arm_navigation_msgs/JointConstraint\n\
# Constrain the position of a joint to be within a certain bound\n\
string joint_name\n\
\n\
# the bound to be achieved is [position - tolerance_below, position + tolerance_above]\n\
float64 position\n\
float64 tolerance_above\n\
float64 tolerance_below\n\
\n\
# A weighting factor for this constraint\n\
float64 weight\n\
================================================================================\n\
MSG: arm_navigation_msgs/PositionConstraint\n\
# This message contains the definition of a position constraint.\n\
Header header\n\
\n\
# The robot link this constraint refers to\n\
string link_name\n\
\n\
# The offset (in the link frame) for the target point on the link we are planning for\n\
geometry_msgs/Point target_point_offset\n\
\n\
# The nominal/target position for the point we are planning for\n\
geometry_msgs/Point position\n\
\n\
# The shape of the bounded region that constrains the position of the end-effector\n\
# This region is always centered at the position defined above\n\
arm_navigation_msgs/Shape constraint_region_shape\n\
\n\
# The orientation of the bounded region that constrains the position of the end-effector. \n\
# This allows the specification of non-axis aligned constraints\n\
geometry_msgs/Quaternion constraint_region_orientation\n\
\n\
# Constraint weighting factor - a weight for this constraint\n\
float64 weight\n\
\n\
================================================================================\n\
MSG: arm_navigation_msgs/OrientationConstraint\n\
# This message contains the definition of an orientation constraint.\n\
Header header\n\
\n\
# The robot link this constraint refers to\n\
string link_name\n\
\n\
# The type of the constraint\n\
int32 type\n\
int32 LINK_FRAME=0\n\
int32 HEADER_FRAME=1\n\
\n\
# The desired orientation of the robot link specified as a quaternion\n\
geometry_msgs/Quaternion orientation\n\
\n\
# optional RPY error tolerances specified if \n\
float64 absolute_roll_tolerance\n\
float64 absolute_pitch_tolerance\n\
float64 absolute_yaw_tolerance\n\
\n\
# Constraint weighting factor - a weight for this constraint\n\
float64 weight\n\
\n\
================================================================================\n\
MSG: arm_navigation_msgs/VisibilityConstraint\n\
# This message contains the definition of a visibility constraint.\n\
Header header\n\
\n\
# The point stamped target that needs to be kept within view of the sensor\n\
geometry_msgs/PointStamped target\n\
\n\
# The local pose of the frame in which visibility is to be maintained\n\
# The frame id should represent the robot link to which the sensor is attached\n\
# The visual axis of the sensor is assumed to be along the X axis of this frame\n\
geometry_msgs/PoseStamped sensor_pose\n\
\n\
# The deviation (in radians) that will be tolerated\n\
# Constraint error will be measured as the solid angle between the \n\
# X axis of the frame defined above and the vector between the origin \n\
# of the frame defined above and the target location\n\
float64 absolute_tolerance\n\
\n\
\n\
================================================================================\n\
MSG: geometry_msgs/PointStamped\n\
# This represents a Point with reference coordinate frame and timestamp\n\
Header header\n\
Point point\n\
\n\
================================================================================\n\
MSG: arm_navigation_msgs/OrderedCollisionOperations\n\
# A set of collision operations that will be performed in the order they are specified\n\
CollisionOperation[] collision_operations\n\
================================================================================\n\
MSG: arm_navigation_msgs/CollisionOperation\n\
# A definition of a collision operation\n\
# E.g. (\"gripper\",COLLISION_SET_ALL,ENABLE) will enable collisions \n\
# between the gripper and all objects in the collision space\n\
\n\
string object1\n\
string object2\n\
string COLLISION_SET_ALL=\"all\"\n\
string COLLISION_SET_OBJECTS=\"objects\"\n\
string COLLISION_SET_ATTACHED_OBJECTS=\"attached\"\n\
\n\
# The penetration distance to which collisions are allowed. This is 0.0 by default.\n\
float64 penetration_distance\n\
\n\
# Flag that determines whether collisions will be enabled or disabled for the pair of objects specified above\n\
int32 operation\n\
int32 DISABLE=0\n\
int32 ENABLE=1\n\
\n\
";
  }

  static const char* value(const  ::arm_navigation_msgs::MoveArmGoal_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::arm_navigation_msgs::MoveArmGoal_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.planner_service_name);
    stream.next(m.planning_scene_diff);
    stream.next(m.motion_plan_request);
    stream.next(m.operations);
    stream.next(m.accept_partial_plans);
    stream.next(m.accept_invalid_goals);
    stream.next(m.disable_ik);
    stream.next(m.disable_collision_monitoring);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct MoveArmGoal_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::arm_navigation_msgs::MoveArmGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::arm_navigation_msgs::MoveArmGoal_<ContainerAllocator> & v) 
  {
    s << indent << "planner_service_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.planner_service_name);
    s << indent << "planning_scene_diff: ";
s << std::endl;
    Printer< ::arm_navigation_msgs::PlanningScene_<ContainerAllocator> >::stream(s, indent + "  ", v.planning_scene_diff);
    s << indent << "motion_plan_request: ";
s << std::endl;
    Printer< ::arm_navigation_msgs::MotionPlanRequest_<ContainerAllocator> >::stream(s, indent + "  ", v.motion_plan_request);
    s << indent << "operations: ";
s << std::endl;
    Printer< ::arm_navigation_msgs::OrderedCollisionOperations_<ContainerAllocator> >::stream(s, indent + "  ", v.operations);
    s << indent << "accept_partial_plans: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.accept_partial_plans);
    s << indent << "accept_invalid_goals: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.accept_invalid_goals);
    s << indent << "disable_ik: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.disable_ik);
    s << indent << "disable_collision_monitoring: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.disable_collision_monitoring);
  }
};


} // namespace message_operations
} // namespace ros

#endif // ARM_NAVIGATION_MSGS_MESSAGE_MOVEARMGOAL_H

