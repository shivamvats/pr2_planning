; Auto-generated. Do not edit!


(cl:in-package arm_navigation_msgs-msg)


;//! \htmlinclude MoveArmResult.msg.html

(cl:defclass <MoveArmResult> (roslisp-msg-protocol:ros-message)
  ((error_code
    :reader error_code
    :initarg :error_code
    :type arm_navigation_msgs-msg:ArmNavigationErrorCodes
    :initform (cl:make-instance 'arm_navigation_msgs-msg:ArmNavigationErrorCodes))
   (contacts
    :reader contacts
    :initarg :contacts
    :type (cl:vector arm_navigation_msgs-msg:ContactInformation)
   :initform (cl:make-array 0 :element-type 'arm_navigation_msgs-msg:ContactInformation :initial-element (cl:make-instance 'arm_navigation_msgs-msg:ContactInformation))))
)

(cl:defclass MoveArmResult (<MoveArmResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveArmResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveArmResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name arm_navigation_msgs-msg:<MoveArmResult> is deprecated: use arm_navigation_msgs-msg:MoveArmResult instead.")))

(cl:ensure-generic-function 'error_code-val :lambda-list '(m))
(cl:defmethod error_code-val ((m <MoveArmResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_navigation_msgs-msg:error_code-val is deprecated.  Use arm_navigation_msgs-msg:error_code instead.")
  (error_code m))

(cl:ensure-generic-function 'contacts-val :lambda-list '(m))
(cl:defmethod contacts-val ((m <MoveArmResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_navigation_msgs-msg:contacts-val is deprecated.  Use arm_navigation_msgs-msg:contacts instead.")
  (contacts m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveArmResult>) ostream)
  "Serializes a message object of type '<MoveArmResult>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'error_code) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'contacts))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'contacts))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveArmResult>) istream)
  "Deserializes a message object of type '<MoveArmResult>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'error_code) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'contacts) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'contacts)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'arm_navigation_msgs-msg:ContactInformation))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveArmResult>)))
  "Returns string type for a message object of type '<MoveArmResult>"
  "arm_navigation_msgs/MoveArmResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveArmResult)))
  "Returns string type for a message object of type 'MoveArmResult"
  "arm_navigation_msgs/MoveArmResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveArmResult>)))
  "Returns md5sum for a message object of type '<MoveArmResult>"
  "3229301226a0605e3ffc9dfdaeac662f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveArmResult)))
  "Returns md5sum for a message object of type 'MoveArmResult"
  "3229301226a0605e3ffc9dfdaeac662f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveArmResult>)))
  "Returns full string definition for message of type '<MoveArmResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# An error code reflecting what went wrong~%ArmNavigationErrorCodes error_code~%~%ContactInformation[] contacts~%~%================================================================================~%MSG: arm_navigation_msgs/ArmNavigationErrorCodes~%int32 val~%~%# overall behavior~%int32 PLANNING_FAILED=-1~%int32 SUCCESS=1~%int32 TIMED_OUT=-2~%~%# start state errors~%int32 START_STATE_IN_COLLISION=-3~%int32 START_STATE_VIOLATES_PATH_CONSTRAINTS=-4~%~%# goal errors~%int32 GOAL_IN_COLLISION=-5~%int32 GOAL_VIOLATES_PATH_CONSTRAINTS=-6~%~%# robot state~%int32 INVALID_ROBOT_STATE=-7~%int32 INCOMPLETE_ROBOT_STATE=-8~%~%# planning request errors~%int32 INVALID_PLANNER_ID=-9~%int32 INVALID_NUM_PLANNING_ATTEMPTS=-10~%int32 INVALID_ALLOWED_PLANNING_TIME=-11~%int32 INVALID_GROUP_NAME=-12~%int32 INVALID_GOAL_JOINT_CONSTRAINTS=-13~%int32 INVALID_GOAL_POSITION_CONSTRAINTS=-14~%int32 INVALID_GOAL_ORIENTATION_CONSTRAINTS=-15~%int32 INVALID_PATH_JOINT_CONSTRAINTS=-16~%int32 INVALID_PATH_POSITION_CONSTRAINTS=-17~%int32 INVALID_PATH_ORIENTATION_CONSTRAINTS=-18~%~%# state/trajectory monitor errors~%int32 INVALID_TRAJECTORY=-19~%int32 INVALID_INDEX=-20~%int32 JOINT_LIMITS_VIOLATED=-21~%int32 PATH_CONSTRAINTS_VIOLATED=-22~%int32 COLLISION_CONSTRAINTS_VIOLATED=-23~%int32 GOAL_CONSTRAINTS_VIOLATED=-24~%int32 JOINTS_NOT_MOVING=-25~%int32 TRAJECTORY_CONTROLLER_FAILED=-26~%~%# system errors~%int32 FRAME_TRANSFORM_FAILURE=-27~%int32 COLLISION_CHECKING_UNAVAILABLE=-28~%int32 ROBOT_STATE_STALE=-29~%int32 SENSOR_INFO_STALE=-30~%~%# kinematics errors~%int32 NO_IK_SOLUTION=-31~%int32 INVALID_LINK_NAME=-32~%int32 IK_LINK_IN_COLLISION=-33~%int32 NO_FK_SOLUTION=-34~%int32 KINEMATICS_STATE_IN_COLLISION=-35~%~%# general errors~%int32 INVALID_TIMEOUT=-36~%~%~%================================================================================~%MSG: arm_navigation_msgs/ContactInformation~%# Standard ROS header contains information ~%# about the frame in which this ~%# contact is specified~%Header header~%~%# Position of the contact point~%geometry_msgs/Point position~%~%# Normal corresponding to the contact point~%geometry_msgs/Vector3 normal ~%~%# Depth of contact point~%float64 depth~%~%# Name of the first body that is in contact~%# This could be a link or a namespace that represents a body~%string contact_body_1~%string attached_body_1~%uint32 body_type_1~%~%# Name of the second body that is in contact~%# This could be a link or a namespace that represents a body~%string contact_body_2~%string attached_body_2~%uint32 body_type_2~%~%uint32 ROBOT_LINK=0~%uint32 OBJECT=1~%uint32 ATTACHED_BODY=2~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveArmResult)))
  "Returns full string definition for message of type 'MoveArmResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# An error code reflecting what went wrong~%ArmNavigationErrorCodes error_code~%~%ContactInformation[] contacts~%~%================================================================================~%MSG: arm_navigation_msgs/ArmNavigationErrorCodes~%int32 val~%~%# overall behavior~%int32 PLANNING_FAILED=-1~%int32 SUCCESS=1~%int32 TIMED_OUT=-2~%~%# start state errors~%int32 START_STATE_IN_COLLISION=-3~%int32 START_STATE_VIOLATES_PATH_CONSTRAINTS=-4~%~%# goal errors~%int32 GOAL_IN_COLLISION=-5~%int32 GOAL_VIOLATES_PATH_CONSTRAINTS=-6~%~%# robot state~%int32 INVALID_ROBOT_STATE=-7~%int32 INCOMPLETE_ROBOT_STATE=-8~%~%# planning request errors~%int32 INVALID_PLANNER_ID=-9~%int32 INVALID_NUM_PLANNING_ATTEMPTS=-10~%int32 INVALID_ALLOWED_PLANNING_TIME=-11~%int32 INVALID_GROUP_NAME=-12~%int32 INVALID_GOAL_JOINT_CONSTRAINTS=-13~%int32 INVALID_GOAL_POSITION_CONSTRAINTS=-14~%int32 INVALID_GOAL_ORIENTATION_CONSTRAINTS=-15~%int32 INVALID_PATH_JOINT_CONSTRAINTS=-16~%int32 INVALID_PATH_POSITION_CONSTRAINTS=-17~%int32 INVALID_PATH_ORIENTATION_CONSTRAINTS=-18~%~%# state/trajectory monitor errors~%int32 INVALID_TRAJECTORY=-19~%int32 INVALID_INDEX=-20~%int32 JOINT_LIMITS_VIOLATED=-21~%int32 PATH_CONSTRAINTS_VIOLATED=-22~%int32 COLLISION_CONSTRAINTS_VIOLATED=-23~%int32 GOAL_CONSTRAINTS_VIOLATED=-24~%int32 JOINTS_NOT_MOVING=-25~%int32 TRAJECTORY_CONTROLLER_FAILED=-26~%~%# system errors~%int32 FRAME_TRANSFORM_FAILURE=-27~%int32 COLLISION_CHECKING_UNAVAILABLE=-28~%int32 ROBOT_STATE_STALE=-29~%int32 SENSOR_INFO_STALE=-30~%~%# kinematics errors~%int32 NO_IK_SOLUTION=-31~%int32 INVALID_LINK_NAME=-32~%int32 IK_LINK_IN_COLLISION=-33~%int32 NO_FK_SOLUTION=-34~%int32 KINEMATICS_STATE_IN_COLLISION=-35~%~%# general errors~%int32 INVALID_TIMEOUT=-36~%~%~%================================================================================~%MSG: arm_navigation_msgs/ContactInformation~%# Standard ROS header contains information ~%# about the frame in which this ~%# contact is specified~%Header header~%~%# Position of the contact point~%geometry_msgs/Point position~%~%# Normal corresponding to the contact point~%geometry_msgs/Vector3 normal ~%~%# Depth of contact point~%float64 depth~%~%# Name of the first body that is in contact~%# This could be a link or a namespace that represents a body~%string contact_body_1~%string attached_body_1~%uint32 body_type_1~%~%# Name of the second body that is in contact~%# This could be a link or a namespace that represents a body~%string contact_body_2~%string attached_body_2~%uint32 body_type_2~%~%uint32 ROBOT_LINK=0~%uint32 OBJECT=1~%uint32 ATTACHED_BODY=2~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveArmResult>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'error_code))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'contacts) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveArmResult>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveArmResult
    (cl:cons ':error_code (error_code msg))
    (cl:cons ':contacts (contacts msg))
))
