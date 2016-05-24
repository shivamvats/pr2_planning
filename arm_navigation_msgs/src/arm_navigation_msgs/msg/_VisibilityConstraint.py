"""autogenerated by genpy from arm_navigation_msgs/VisibilityConstraint.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import std_msgs.msg

class VisibilityConstraint(genpy.Message):
  _md5sum = "ab297b6588ea21c1a862067d8447cb08"
  _type = "arm_navigation_msgs/VisibilityConstraint"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """# This message contains the definition of a visibility constraint.
Header header

# The point stamped target that needs to be kept within view of the sensor
geometry_msgs/PointStamped target

# The local pose of the frame in which visibility is to be maintained
# The frame id should represent the robot link to which the sensor is attached
# The visual axis of the sensor is assumed to be along the X axis of this frame
geometry_msgs/PoseStamped sensor_pose

# The deviation (in radians) that will be tolerated
# Constraint error will be measured as the solid angle between the 
# X axis of the frame defined above and the vector between the origin 
# of the frame defined above and the target location
float64 absolute_tolerance


================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/PointStamped
# This represents a Point with reference coordinate frame and timestamp
Header header
Point point

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of postion and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

"""
  __slots__ = ['header','target','sensor_pose','absolute_tolerance']
  _slot_types = ['std_msgs/Header','geometry_msgs/PointStamped','geometry_msgs/PoseStamped','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,target,sensor_pose,absolute_tolerance

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(VisibilityConstraint, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.target is None:
        self.target = geometry_msgs.msg.PointStamped()
      if self.sensor_pose is None:
        self.sensor_pose = geometry_msgs.msg.PoseStamped()
      if self.absolute_tolerance is None:
        self.absolute_tolerance = 0.
    else:
      self.header = std_msgs.msg.Header()
      self.target = geometry_msgs.msg.PointStamped()
      self.sensor_pose = geometry_msgs.msg.PoseStamped()
      self.absolute_tolerance = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3I.pack(_x.target.header.seq, _x.target.header.stamp.secs, _x.target.header.stamp.nsecs))
      _x = self.target.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3d3I.pack(_x.target.point.x, _x.target.point.y, _x.target.point.z, _x.sensor_pose.header.seq, _x.sensor_pose.header.stamp.secs, _x.sensor_pose.header.stamp.nsecs))
      _x = self.sensor_pose.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_8d.pack(_x.sensor_pose.pose.position.x, _x.sensor_pose.pose.position.y, _x.sensor_pose.pose.position.z, _x.sensor_pose.pose.orientation.x, _x.sensor_pose.pose.orientation.y, _x.sensor_pose.pose.orientation.z, _x.sensor_pose.pose.orientation.w, _x.absolute_tolerance))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.target is None:
        self.target = geometry_msgs.msg.PointStamped()
      if self.sensor_pose is None:
        self.sensor_pose = geometry_msgs.msg.PoseStamped()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.target.header.seq, _x.target.header.stamp.secs, _x.target.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.target.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.target.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 36
      (_x.target.point.x, _x.target.point.y, _x.target.point.z, _x.sensor_pose.header.seq, _x.sensor_pose.header.stamp.secs, _x.sensor_pose.header.stamp.nsecs,) = _struct_3d3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.sensor_pose.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.sensor_pose.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 64
      (_x.sensor_pose.pose.position.x, _x.sensor_pose.pose.position.y, _x.sensor_pose.pose.position.z, _x.sensor_pose.pose.orientation.x, _x.sensor_pose.pose.orientation.y, _x.sensor_pose.pose.orientation.z, _x.sensor_pose.pose.orientation.w, _x.absolute_tolerance,) = _struct_8d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3I.pack(_x.target.header.seq, _x.target.header.stamp.secs, _x.target.header.stamp.nsecs))
      _x = self.target.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3d3I.pack(_x.target.point.x, _x.target.point.y, _x.target.point.z, _x.sensor_pose.header.seq, _x.sensor_pose.header.stamp.secs, _x.sensor_pose.header.stamp.nsecs))
      _x = self.sensor_pose.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_8d.pack(_x.sensor_pose.pose.position.x, _x.sensor_pose.pose.position.y, _x.sensor_pose.pose.position.z, _x.sensor_pose.pose.orientation.x, _x.sensor_pose.pose.orientation.y, _x.sensor_pose.pose.orientation.z, _x.sensor_pose.pose.orientation.w, _x.absolute_tolerance))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.target is None:
        self.target = geometry_msgs.msg.PointStamped()
      if self.sensor_pose is None:
        self.sensor_pose = geometry_msgs.msg.PoseStamped()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.target.header.seq, _x.target.header.stamp.secs, _x.target.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.target.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.target.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 36
      (_x.target.point.x, _x.target.point.y, _x.target.point.z, _x.sensor_pose.header.seq, _x.sensor_pose.header.stamp.secs, _x.sensor_pose.header.stamp.nsecs,) = _struct_3d3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.sensor_pose.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.sensor_pose.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 64
      (_x.sensor_pose.pose.position.x, _x.sensor_pose.pose.position.y, _x.sensor_pose.pose.position.z, _x.sensor_pose.pose.orientation.x, _x.sensor_pose.pose.orientation.y, _x.sensor_pose.pose.orientation.z, _x.sensor_pose.pose.orientation.w, _x.absolute_tolerance,) = _struct_8d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3d3I = struct.Struct("<3d3I")
_struct_3I = struct.Struct("<3I")
_struct_8d = struct.Struct("<8d")