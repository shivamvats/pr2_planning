/* Auto-generated by genmsg_cpp for file /home/osboxes/groovy_ws/sandbox/arm_navigation_msgs/arm_navigation_msgs/srv/SetPlanningSceneDiff.srv */
#ifndef ARM_NAVIGATION_MSGS_SERVICE_SETPLANNINGSCENEDIFF_H
#define ARM_NAVIGATION_MSGS_SERVICE_SETPLANNINGSCENEDIFF_H
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

#include "arm_navigation_msgs/PlanningScene.h"
#include "arm_navigation_msgs/OrderedCollisionOperations.h"


#include "arm_navigation_msgs/PlanningScene.h"

namespace arm_navigation_msgs
{
template <class ContainerAllocator>
struct SetPlanningSceneDiffRequest_ {
  typedef SetPlanningSceneDiffRequest_<ContainerAllocator> Type;

  SetPlanningSceneDiffRequest_()
  : planning_scene_diff()
  , operations()
  {
  }

  SetPlanningSceneDiffRequest_(const ContainerAllocator& _alloc)
  : planning_scene_diff(_alloc)
  , operations(_alloc)
  {
  }

  typedef  ::arm_navigation_msgs::PlanningScene_<ContainerAllocator>  _planning_scene_diff_type;
   ::arm_navigation_msgs::PlanningScene_<ContainerAllocator>  planning_scene_diff;

  typedef  ::arm_navigation_msgs::OrderedCollisionOperations_<ContainerAllocator>  _operations_type;
   ::arm_navigation_msgs::OrderedCollisionOperations_<ContainerAllocator>  operations;


  typedef boost::shared_ptr< ::arm_navigation_msgs::SetPlanningSceneDiffRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::arm_navigation_msgs::SetPlanningSceneDiffRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SetPlanningSceneDiffRequest
typedef  ::arm_navigation_msgs::SetPlanningSceneDiffRequest_<std::allocator<void> > SetPlanningSceneDiffRequest;

typedef boost::shared_ptr< ::arm_navigation_msgs::SetPlanningSceneDiffRequest> SetPlanningSceneDiffRequestPtr;
typedef boost::shared_ptr< ::arm_navigation_msgs::SetPlanningSceneDiffRequest const> SetPlanningSceneDiffRequestConstPtr;



template <class ContainerAllocator>
struct SetPlanningSceneDiffResponse_ {
  typedef SetPlanningSceneDiffResponse_<ContainerAllocator> Type;

  SetPlanningSceneDiffResponse_()
  : planning_scene()
  {
  }

  SetPlanningSceneDiffResponse_(const ContainerAllocator& _alloc)
  : planning_scene(_alloc)
  {
  }

  typedef  ::arm_navigation_msgs::PlanningScene_<ContainerAllocator>  _planning_scene_type;
   ::arm_navigation_msgs::PlanningScene_<ContainerAllocator>  planning_scene;


  typedef boost::shared_ptr< ::arm_navigation_msgs::SetPlanningSceneDiffResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::arm_navigation_msgs::SetPlanningSceneDiffResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct SetPlanningSceneDiffResponse
typedef  ::arm_navigation_msgs::SetPlanningSceneDiffResponse_<std::allocator<void> > SetPlanningSceneDiffResponse;

typedef boost::shared_ptr< ::arm_navigation_msgs::SetPlanningSceneDiffResponse> SetPlanningSceneDiffResponsePtr;
typedef boost::shared_ptr< ::arm_navigation_msgs::SetPlanningSceneDiffResponse const> SetPlanningSceneDiffResponseConstPtr;


struct SetPlanningSceneDiff
{

typedef SetPlanningSceneDiffRequest Request;
typedef SetPlanningSceneDiffResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct SetPlanningSceneDiff
} // namespace arm_navigation_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::arm_navigation_msgs::SetPlanningSceneDiffRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::arm_navigation_msgs::SetPlanningSceneDiffRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::arm_navigation_msgs::SetPlanningSceneDiffRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "67ad55e9bed9c8f21dfb4b9b1ca8df7d";
  }

  static const char* value(const  ::arm_navigation_msgs::SetPlanningSceneDiffRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x67ad55e9bed9c8f2ULL;
  static const uint64_t static_value2 = 0x1dfb4b9b1ca8df7dULL;
};

template<class ContainerAllocator>
struct DataType< ::arm_navigation_msgs::SetPlanningSceneDiffRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "arm_navigation_msgs/SetPlanningSceneDiffRequest";
  }

  static const char* value(const  ::arm_navigation_msgs::SetPlanningSceneDiffRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::arm_navigation_msgs::SetPlanningSceneDiffRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
PlanningScene planning_scene_diff\n\
\n\
\n\
arm_navigation_msgs/OrderedCollisionOperations operations\n\
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

  static const char* value(const  ::arm_navigation_msgs::SetPlanningSceneDiffRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::arm_navigation_msgs::SetPlanningSceneDiffResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::arm_navigation_msgs::SetPlanningSceneDiffResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::arm_navigation_msgs::SetPlanningSceneDiffResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "285525c9abe002fbafa99af84a14b4cb";
  }

  static const char* value(const  ::arm_navigation_msgs::SetPlanningSceneDiffResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x285525c9abe002fbULL;
  static const uint64_t static_value2 = 0xafa99af84a14b4cbULL;
};

template<class ContainerAllocator>
struct DataType< ::arm_navigation_msgs::SetPlanningSceneDiffResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "arm_navigation_msgs/SetPlanningSceneDiffResponse";
  }

  static const char* value(const  ::arm_navigation_msgs::SetPlanningSceneDiffResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::arm_navigation_msgs::SetPlanningSceneDiffResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
PlanningScene planning_scene\n\
\n\
\n\
\n\
\n\
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
";
  }

  static const char* value(const  ::arm_navigation_msgs::SetPlanningSceneDiffResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::arm_navigation_msgs::SetPlanningSceneDiffRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.planning_scene_diff);
    stream.next(m.operations);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SetPlanningSceneDiffRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::arm_navigation_msgs::SetPlanningSceneDiffResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.planning_scene);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct SetPlanningSceneDiffResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<arm_navigation_msgs::SetPlanningSceneDiff> {
  static const char* value() 
  {
    return "0a7b07718e4e5c5d35740c730509a151";
  }

  static const char* value(const arm_navigation_msgs::SetPlanningSceneDiff&) { return value(); } 
};

template<>
struct DataType<arm_navigation_msgs::SetPlanningSceneDiff> {
  static const char* value() 
  {
    return "arm_navigation_msgs/SetPlanningSceneDiff";
  }

  static const char* value(const arm_navigation_msgs::SetPlanningSceneDiff&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<arm_navigation_msgs::SetPlanningSceneDiffRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "0a7b07718e4e5c5d35740c730509a151";
  }

  static const char* value(const arm_navigation_msgs::SetPlanningSceneDiffRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<arm_navigation_msgs::SetPlanningSceneDiffRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "arm_navigation_msgs/SetPlanningSceneDiff";
  }

  static const char* value(const arm_navigation_msgs::SetPlanningSceneDiffRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<arm_navigation_msgs::SetPlanningSceneDiffResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "0a7b07718e4e5c5d35740c730509a151";
  }

  static const char* value(const arm_navigation_msgs::SetPlanningSceneDiffResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<arm_navigation_msgs::SetPlanningSceneDiffResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "arm_navigation_msgs/SetPlanningSceneDiff";
  }

  static const char* value(const arm_navigation_msgs::SetPlanningSceneDiffResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // ARM_NAVIGATION_MSGS_SERVICE_SETPLANNINGSCENEDIFF_H

