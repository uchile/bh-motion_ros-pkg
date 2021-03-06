"""autogenerated by genpy from bh_motion/Pose3D.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import bh_motion.msg

class Pose3D(genpy.Message):
  _md5sum = "86faefed96f939581fe9bb0ec87e5f9a"
  _type = "bh_motion/Pose3D"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """RotationMatrix rotation

#/** translation as a Vector3*/
Vector3 translation

================================================================================
MSG: bh_motion/RotationMatrix
#  * \param  c0  the first column of the matrix.
#  * \param  c1  the second column of the matrix.
#  * \param  c2  the third column of the matrix.

Vector3 c0
Vector3 c1
Vector3 c2



================================================================================
MSG: bh_motion/Vector3
float32 x 
float32 y
float32 z

"""
  __slots__ = ['rotation','translation']
  _slot_types = ['bh_motion/RotationMatrix','bh_motion/Vector3']

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
      super(Pose3D, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.rotation is None:
        self.rotation = bh_motion.msg.RotationMatrix()
      if self.translation is None:
        self.translation = bh_motion.msg.Vector3()
    else:
      self.rotation = bh_motion.msg.RotationMatrix()
      self.translation = bh_motion.msg.Vector3()

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
      buff.write(_struct_12f.pack(_x.rotation.c0.x, _x.rotation.c0.y, _x.rotation.c0.z, _x.rotation.c1.x, _x.rotation.c1.y, _x.rotation.c1.z, _x.rotation.c2.x, _x.rotation.c2.y, _x.rotation.c2.z, _x.translation.x, _x.translation.y, _x.translation.z))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.rotation is None:
        self.rotation = bh_motion.msg.RotationMatrix()
      if self.translation is None:
        self.translation = bh_motion.msg.Vector3()
      end = 0
      _x = self
      start = end
      end += 48
      (_x.rotation.c0.x, _x.rotation.c0.y, _x.rotation.c0.z, _x.rotation.c1.x, _x.rotation.c1.y, _x.rotation.c1.z, _x.rotation.c2.x, _x.rotation.c2.y, _x.rotation.c2.z, _x.translation.x, _x.translation.y, _x.translation.z,) = _struct_12f.unpack(str[start:end])
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
      buff.write(_struct_12f.pack(_x.rotation.c0.x, _x.rotation.c0.y, _x.rotation.c0.z, _x.rotation.c1.x, _x.rotation.c1.y, _x.rotation.c1.z, _x.rotation.c2.x, _x.rotation.c2.y, _x.rotation.c2.z, _x.translation.x, _x.translation.y, _x.translation.z))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.rotation is None:
        self.rotation = bh_motion.msg.RotationMatrix()
      if self.translation is None:
        self.translation = bh_motion.msg.Vector3()
      end = 0
      _x = self
      start = end
      end += 48
      (_x.rotation.c0.x, _x.rotation.c0.y, _x.rotation.c0.z, _x.rotation.c1.x, _x.rotation.c1.y, _x.rotation.c1.z, _x.rotation.c2.x, _x.rotation.c2.y, _x.rotation.c2.z, _x.translation.x, _x.translation.y, _x.translation.z,) = _struct_12f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_12f = struct.Struct("<12f")
