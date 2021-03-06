"""autogenerated by genpy from bh_motion/DynPoint.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import bh_motion.msg

class DynPoint(genpy.Message):
  _md5sum = "ff7030c635df334d86113e51daeb9a50"
  _type = "bh_motion/DynPoint"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """uint32 limb
uint32 phaseNumber
uint32 duration
Vector3 translation
Vector3 angle
Vector3 odometryOffset

================================================================================
MSG: bh_motion/Vector3
float32 x 
float32 y
float32 z

"""
  __slots__ = ['limb','phaseNumber','duration','translation','angle','odometryOffset']
  _slot_types = ['uint32','uint32','uint32','bh_motion/Vector3','bh_motion/Vector3','bh_motion/Vector3']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       limb,phaseNumber,duration,translation,angle,odometryOffset

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(DynPoint, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.limb is None:
        self.limb = 0
      if self.phaseNumber is None:
        self.phaseNumber = 0
      if self.duration is None:
        self.duration = 0
      if self.translation is None:
        self.translation = bh_motion.msg.Vector3()
      if self.angle is None:
        self.angle = bh_motion.msg.Vector3()
      if self.odometryOffset is None:
        self.odometryOffset = bh_motion.msg.Vector3()
    else:
      self.limb = 0
      self.phaseNumber = 0
      self.duration = 0
      self.translation = bh_motion.msg.Vector3()
      self.angle = bh_motion.msg.Vector3()
      self.odometryOffset = bh_motion.msg.Vector3()

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
      buff.write(_struct_3I9f.pack(_x.limb, _x.phaseNumber, _x.duration, _x.translation.x, _x.translation.y, _x.translation.z, _x.angle.x, _x.angle.y, _x.angle.z, _x.odometryOffset.x, _x.odometryOffset.y, _x.odometryOffset.z))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.translation is None:
        self.translation = bh_motion.msg.Vector3()
      if self.angle is None:
        self.angle = bh_motion.msg.Vector3()
      if self.odometryOffset is None:
        self.odometryOffset = bh_motion.msg.Vector3()
      end = 0
      _x = self
      start = end
      end += 48
      (_x.limb, _x.phaseNumber, _x.duration, _x.translation.x, _x.translation.y, _x.translation.z, _x.angle.x, _x.angle.y, _x.angle.z, _x.odometryOffset.x, _x.odometryOffset.y, _x.odometryOffset.z,) = _struct_3I9f.unpack(str[start:end])
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
      buff.write(_struct_3I9f.pack(_x.limb, _x.phaseNumber, _x.duration, _x.translation.x, _x.translation.y, _x.translation.z, _x.angle.x, _x.angle.y, _x.angle.z, _x.odometryOffset.x, _x.odometryOffset.y, _x.odometryOffset.z))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.translation is None:
        self.translation = bh_motion.msg.Vector3()
      if self.angle is None:
        self.angle = bh_motion.msg.Vector3()
      if self.odometryOffset is None:
        self.odometryOffset = bh_motion.msg.Vector3()
      end = 0
      _x = self
      start = end
      end += 48
      (_x.limb, _x.phaseNumber, _x.duration, _x.translation.x, _x.translation.y, _x.translation.z, _x.angle.x, _x.angle.y, _x.angle.z, _x.odometryOffset.x, _x.odometryOffset.y, _x.odometryOffset.z,) = _struct_3I9f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3I9f = struct.Struct("<3I9f")
