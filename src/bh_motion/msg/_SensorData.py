"""autogenerated by genpy from bh_motion/SensorData.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class SensorData(genpy.Message):
  _md5sum = "da2d1d48d127d312b95c54ba5320a2d9"
  _type = "bh_motion/SensorData"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float32[19] data #numOfSensors
int16[22] currents #numOfJoints
uint8[22] temperatures #numOfJoints
uint32 timeStamp
uint32 usActuatorMode
uint32 usTimeStamp

"""
  __slots__ = ['data','currents','temperatures','timeStamp','usActuatorMode','usTimeStamp']
  _slot_types = ['float32[19]','int16[22]','uint8[22]','uint32','uint32','uint32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       data,currents,temperatures,timeStamp,usActuatorMode,usTimeStamp

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SensorData, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.data is None:
        self.data = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
      if self.currents is None:
        self.currents = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
      if self.temperatures is None:
        self.temperatures = chr(0)*22
      if self.timeStamp is None:
        self.timeStamp = 0
      if self.usActuatorMode is None:
        self.usActuatorMode = 0
      if self.usTimeStamp is None:
        self.usTimeStamp = 0
    else:
      self.data = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
      self.currents = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
      self.temperatures = chr(0)*22
      self.timeStamp = 0
      self.usActuatorMode = 0
      self.usTimeStamp = 0

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
      buff.write(_struct_19f.pack(*self.data))
      buff.write(_struct_22h.pack(*self.currents))
      _x = self.temperatures
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_struct_22B.pack(*_x))
      else:
        buff.write(_struct_22s.pack(_x))
      _x = self
      buff.write(_struct_3I.pack(_x.timeStamp, _x.usActuatorMode, _x.usTimeStamp))
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
      end += 76
      self.data = _struct_19f.unpack(str[start:end])
      start = end
      end += 44
      self.currents = _struct_22h.unpack(str[start:end])
      start = end
      end += 22
      self.temperatures = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.timeStamp, _x.usActuatorMode, _x.usTimeStamp,) = _struct_3I.unpack(str[start:end])
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
      buff.write(self.data.tostring())
      buff.write(self.currents.tostring())
      _x = self.temperatures
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(_struct_22B.pack(*_x))
      else:
        buff.write(_struct_22s.pack(_x))
      _x = self
      buff.write(_struct_3I.pack(_x.timeStamp, _x.usActuatorMode, _x.usTimeStamp))
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
      end += 76
      self.data = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=19)
      start = end
      end += 44
      self.currents = numpy.frombuffer(str[start:end], dtype=numpy.int16, count=22)
      start = end
      end += 22
      self.temperatures = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.timeStamp, _x.usActuatorMode, _x.usTimeStamp,) = _struct_3I.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3I = struct.Struct("<3I")
_struct_19f = struct.Struct("<19f")
_struct_22s = struct.Struct("<22s")
_struct_22B = struct.Struct("<22B")
_struct_22h = struct.Struct("<22h")