"""autogenerated by genpy from bh_motion/TorsoMatrix.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import bh_motion.msg

class TorsoMatrix(genpy.Message):
  _md5sum = "aae78ffd263259e8e641ca9080b789ed"
  _type = "bh_motion/TorsoMatrix"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """Pose3D offset   # /**< The estimated offset (including odometry) from last torso matrix to this one. (relative to the torso) */
bool isValid

================================================================================
MSG: bh_motion/Pose3D
RotationMatrix rotation

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
  __slots__ = ['offset','isValid']
  _slot_types = ['bh_motion/Pose3D','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       offset,isValid

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(TorsoMatrix, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.offset is None:
        self.offset = bh_motion.msg.Pose3D()
      if self.isValid is None:
        self.isValid = False
    else:
      self.offset = bh_motion.msg.Pose3D()
      self.isValid = False

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
      buff.write(_struct_12fB.pack(_x.offset.rotation.c0.x, _x.offset.rotation.c0.y, _x.offset.rotation.c0.z, _x.offset.rotation.c1.x, _x.offset.rotation.c1.y, _x.offset.rotation.c1.z, _x.offset.rotation.c2.x, _x.offset.rotation.c2.y, _x.offset.rotation.c2.z, _x.offset.translation.x, _x.offset.translation.y, _x.offset.translation.z, _x.isValid))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.offset is None:
        self.offset = bh_motion.msg.Pose3D()
      end = 0
      _x = self
      start = end
      end += 49
      (_x.offset.rotation.c0.x, _x.offset.rotation.c0.y, _x.offset.rotation.c0.z, _x.offset.rotation.c1.x, _x.offset.rotation.c1.y, _x.offset.rotation.c1.z, _x.offset.rotation.c2.x, _x.offset.rotation.c2.y, _x.offset.rotation.c2.z, _x.offset.translation.x, _x.offset.translation.y, _x.offset.translation.z, _x.isValid,) = _struct_12fB.unpack(str[start:end])
      self.isValid = bool(self.isValid)
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
      buff.write(_struct_12fB.pack(_x.offset.rotation.c0.x, _x.offset.rotation.c0.y, _x.offset.rotation.c0.z, _x.offset.rotation.c1.x, _x.offset.rotation.c1.y, _x.offset.rotation.c1.z, _x.offset.rotation.c2.x, _x.offset.rotation.c2.y, _x.offset.rotation.c2.z, _x.offset.translation.x, _x.offset.translation.y, _x.offset.translation.z, _x.isValid))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.offset is None:
        self.offset = bh_motion.msg.Pose3D()
      end = 0
      _x = self
      start = end
      end += 49
      (_x.offset.rotation.c0.x, _x.offset.rotation.c0.y, _x.offset.rotation.c0.z, _x.offset.rotation.c1.x, _x.offset.rotation.c1.y, _x.offset.rotation.c1.z, _x.offset.rotation.c2.x, _x.offset.rotation.c2.y, _x.offset.rotation.c2.z, _x.offset.translation.x, _x.offset.translation.y, _x.offset.translation.z, _x.isValid,) = _struct_12fB.unpack(str[start:end])
      self.isValid = bool(self.isValid)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_12fB = struct.Struct("<12fB")
