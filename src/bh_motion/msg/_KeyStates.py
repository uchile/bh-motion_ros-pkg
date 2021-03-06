"""autogenerated by genpy from bh_motion/KeyStates.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class KeyStates(genpy.Message):
  _md5sum = "6cdbdbfd8d859df1fae436c0707372cf"
  _type = "bh_motion/KeyStates"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """#ENUM(Key: rightFootRight, rightFootLeft, leftFootRight, leftFootLeft, chest);


bool[5] pressed

"""
  __slots__ = ['pressed']
  _slot_types = ['bool[5]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       pressed

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(KeyStates, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.pressed is None:
        self.pressed = [False,False,False,False,False]
    else:
      self.pressed = [False,False,False,False,False]

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
      buff.write(_struct_5B.pack(*self.pressed))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 5
      self.pressed = _struct_5B.unpack(str[start:end])
      self.pressed = map(bool, self.pressed)
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
      buff.write(self.pressed.tostring())
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 5
      self.pressed = numpy.frombuffer(str[start:end], dtype=numpy.bool, count=5)
      self.pressed = map(bool, self.pressed)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_5B = struct.Struct("<5B")
