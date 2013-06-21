"""autogenerated by genpy from bh_motion/Pose2D.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import bh_motion.msg

class Pose2D(genpy.Message):
  _md5sum = "6c1f64b87a2b26242101a64f09d84eac"
  _type = "bh_motion/Pose2D"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# This expresses a position and orientation on a 2D (rotation, transaltion.x, transation.y).

float32 rotation
Translation translation

================================================================================
MSG: bh_motion/Translation
# This expresses an x, y translation

float32 x
float32 y

"""
  __slots__ = ['rotation','translation']
  _slot_types = ['float32','bh_motion/Translation']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       rotation,translation

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Pose2D, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.rotation is None:
        self.rotation = 0.
      if self.translation is None:
        self.translation = bh_motion.msg.Translation()
    else:
      self.rotation = 0.
      self.translation = bh_motion.msg.Translation()

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
      buff.write(_struct_3f.pack(_x.rotation, _x.translation.x, _x.translation.y))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.translation is None:
        self.translation = bh_motion.msg.Translation()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.rotation, _x.translation.x, _x.translation.y,) = _struct_3f.unpack(str[start:end])
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
      buff.write(_struct_3f.pack(_x.rotation, _x.translation.x, _x.translation.y))
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
        self.translation = bh_motion.msg.Translation()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.rotation, _x.translation.x, _x.translation.y,) = _struct_3f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3f = struct.Struct("<3f")