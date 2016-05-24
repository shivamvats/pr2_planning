"""autogenerated by genpy from arm_navigation_msgs/OrientedBoundingBox.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg

class OrientedBoundingBox(genpy.Message):
  _md5sum = "a9b13162620bd04a7cb84cf207e7a8ac"
  _type = "arm_navigation_msgs/OrientedBoundingBox"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """#the center of the box
geometry_msgs/Point32 center

#the extents of the box, assuming the center is at the point
geometry_msgs/Point32 extents

#the axis of the box
geometry_msgs/Point32 axis

#the angle of rotation around the axis
float32 angle

================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommeded to use Point wherever possible instead of Point32.  
# 
# This recommendation is to promote interoperability.  
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.  

float32 x
float32 y
float32 z
"""
  __slots__ = ['center','extents','axis','angle']
  _slot_types = ['geometry_msgs/Point32','geometry_msgs/Point32','geometry_msgs/Point32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       center,extents,axis,angle

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(OrientedBoundingBox, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.center is None:
        self.center = geometry_msgs.msg.Point32()
      if self.extents is None:
        self.extents = geometry_msgs.msg.Point32()
      if self.axis is None:
        self.axis = geometry_msgs.msg.Point32()
      if self.angle is None:
        self.angle = 0.
    else:
      self.center = geometry_msgs.msg.Point32()
      self.extents = geometry_msgs.msg.Point32()
      self.axis = geometry_msgs.msg.Point32()
      self.angle = 0.

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
      buff.write(_struct_10f.pack(_x.center.x, _x.center.y, _x.center.z, _x.extents.x, _x.extents.y, _x.extents.z, _x.axis.x, _x.axis.y, _x.axis.z, _x.angle))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.center is None:
        self.center = geometry_msgs.msg.Point32()
      if self.extents is None:
        self.extents = geometry_msgs.msg.Point32()
      if self.axis is None:
        self.axis = geometry_msgs.msg.Point32()
      end = 0
      _x = self
      start = end
      end += 40
      (_x.center.x, _x.center.y, _x.center.z, _x.extents.x, _x.extents.y, _x.extents.z, _x.axis.x, _x.axis.y, _x.axis.z, _x.angle,) = _struct_10f.unpack(str[start:end])
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
      buff.write(_struct_10f.pack(_x.center.x, _x.center.y, _x.center.z, _x.extents.x, _x.extents.y, _x.extents.z, _x.axis.x, _x.axis.y, _x.axis.z, _x.angle))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.center is None:
        self.center = geometry_msgs.msg.Point32()
      if self.extents is None:
        self.extents = geometry_msgs.msg.Point32()
      if self.axis is None:
        self.axis = geometry_msgs.msg.Point32()
      end = 0
      _x = self
      start = end
      end += 40
      (_x.center.x, _x.center.y, _x.center.z, _x.extents.x, _x.extents.y, _x.extents.z, _x.axis.x, _x.axis.y, _x.axis.z, _x.angle,) = _struct_10f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_10f = struct.Struct("<10f")
