; Auto-generated. Do not edit!


(cl:in-package arm_navigation_msgs-msg)


;//! \htmlinclude OrientedBoundingBox.msg.html

(cl:defclass <OrientedBoundingBox> (roslisp-msg-protocol:ros-message)
  ((center
    :reader center
    :initarg :center
    :type geometry_msgs-msg:Point32
    :initform (cl:make-instance 'geometry_msgs-msg:Point32))
   (extents
    :reader extents
    :initarg :extents
    :type geometry_msgs-msg:Point32
    :initform (cl:make-instance 'geometry_msgs-msg:Point32))
   (axis
    :reader axis
    :initarg :axis
    :type geometry_msgs-msg:Point32
    :initform (cl:make-instance 'geometry_msgs-msg:Point32))
   (angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass OrientedBoundingBox (<OrientedBoundingBox>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OrientedBoundingBox>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OrientedBoundingBox)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name arm_navigation_msgs-msg:<OrientedBoundingBox> is deprecated: use arm_navigation_msgs-msg:OrientedBoundingBox instead.")))

(cl:ensure-generic-function 'center-val :lambda-list '(m))
(cl:defmethod center-val ((m <OrientedBoundingBox>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_navigation_msgs-msg:center-val is deprecated.  Use arm_navigation_msgs-msg:center instead.")
  (center m))

(cl:ensure-generic-function 'extents-val :lambda-list '(m))
(cl:defmethod extents-val ((m <OrientedBoundingBox>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_navigation_msgs-msg:extents-val is deprecated.  Use arm_navigation_msgs-msg:extents instead.")
  (extents m))

(cl:ensure-generic-function 'axis-val :lambda-list '(m))
(cl:defmethod axis-val ((m <OrientedBoundingBox>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_navigation_msgs-msg:axis-val is deprecated.  Use arm_navigation_msgs-msg:axis instead.")
  (axis m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <OrientedBoundingBox>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader arm_navigation_msgs-msg:angle-val is deprecated.  Use arm_navigation_msgs-msg:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OrientedBoundingBox>) ostream)
  "Serializes a message object of type '<OrientedBoundingBox>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'center) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'extents) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'axis) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OrientedBoundingBox>) istream)
  "Deserializes a message object of type '<OrientedBoundingBox>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'center) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'extents) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'axis) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OrientedBoundingBox>)))
  "Returns string type for a message object of type '<OrientedBoundingBox>"
  "arm_navigation_msgs/OrientedBoundingBox")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OrientedBoundingBox)))
  "Returns string type for a message object of type 'OrientedBoundingBox"
  "arm_navigation_msgs/OrientedBoundingBox")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OrientedBoundingBox>)))
  "Returns md5sum for a message object of type '<OrientedBoundingBox>"
  "a9b13162620bd04a7cb84cf207e7a8ac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OrientedBoundingBox)))
  "Returns md5sum for a message object of type 'OrientedBoundingBox"
  "a9b13162620bd04a7cb84cf207e7a8ac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OrientedBoundingBox>)))
  "Returns full string definition for message of type '<OrientedBoundingBox>"
  (cl:format cl:nil "#the center of the box~%geometry_msgs/Point32 center~%~%#the extents of the box, assuming the center is at the point~%geometry_msgs/Point32 extents~%~%#the axis of the box~%geometry_msgs/Point32 axis~%~%#the angle of rotation around the axis~%float32 angle~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OrientedBoundingBox)))
  "Returns full string definition for message of type 'OrientedBoundingBox"
  (cl:format cl:nil "#the center of the box~%geometry_msgs/Point32 center~%~%#the extents of the box, assuming the center is at the point~%geometry_msgs/Point32 extents~%~%#the axis of the box~%geometry_msgs/Point32 axis~%~%#the angle of rotation around the axis~%float32 angle~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OrientedBoundingBox>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'center))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'extents))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'axis))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OrientedBoundingBox>))
  "Converts a ROS message object to a list"
  (cl:list 'OrientedBoundingBox
    (cl:cons ':center (center msg))
    (cl:cons ':extents (extents msg))
    (cl:cons ':axis (axis msg))
    (cl:cons ':angle (angle msg))
))
