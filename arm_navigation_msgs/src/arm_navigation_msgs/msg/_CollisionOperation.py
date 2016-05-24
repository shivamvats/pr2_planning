"""autogenerated by genpy from arm_navigation_msgs/CollisionOperation.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class CollisionOperation(genpy.Message):
  _md5sum = "e0cf3073b26bd86266c918a0c779f8a2"
  _type = "arm_navigation_msgs/CollisionOperation"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# A definition of a collision operation
# E.g. ("gripper",COLLISION_SET_ALL,ENABLE) will enable collisions 
# between the gripper and all objects in the collision space

string object1
string object2
string COLLISION_SET_ALL="all"
string COLLISION_SET_OBJECTS="objects"
string COLLISION_SET_ATTACHED_OBJECTS="attached"

# The penetration distance to which collisions are allowed. This is 0.0 by default.
float64 penetration_distance

# Flag that determines whether collisions will be enabled or disabled for the pair of objects specified above
int32 operation
int32 DISABLE=0
int32 ENABLE=1

"""
  # Pseudo-constants
  COLLISION_SET_ALL = r'"all"'
  COLLISION_SET_OBJECTS = r'"objects"'
  COLLISION_SET_ATTACHED_OBJECTS = r'"attached"'
  DISABLE = 0
  ENABLE = 1

  __slots__ = ['object1','object2','penetration_distance','operation']
  _slot_types = ['string','string','float64','int32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       object1,object2,penetration_distance,operation

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(CollisionOperation, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.object1 is None:
        self.object1 = ''
      if self.object2 is None:
        self.object2 = ''
      if self.penetration_distance is None:
        self.penetration_distance = 0.
      if self.operation is None:
        self.operation = 0
    else:
      self.object1 = ''
      self.object2 = ''
      self.penetration_distance = 0.
      self.operation = 0

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
      _x = self.object1
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.object2
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_di.pack(_x.penetration_distance, _x.operation))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.object1 = str[start:end].decode('utf-8')
      else:
        self.object1 = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.object2 = str[start:end].decode('utf-8')
      else:
        self.object2 = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.penetration_distance, _x.operation,) = _struct_di.unpack(str[start:end])
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
      _x = self.object1
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.object2
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_di.pack(_x.penetration_distance, _x.operation))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.object1 = str[start:end].decode('utf-8')
      else:
        self.object1 = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.object2 = str[start:end].decode('utf-8')
      else:
        self.object2 = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.penetration_distance, _x.operation,) = _struct_di.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_di = struct.Struct("<di")
