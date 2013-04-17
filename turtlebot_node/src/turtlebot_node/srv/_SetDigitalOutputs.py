"""autogenerated by genpy from turtlebot_node/SetDigitalOutputsRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class SetDigitalOutputsRequest(genpy.Message):
  _md5sum = "95ef1ce60d04abfe27bea339a6261f29"
  _type = "turtlebot_node/SetDigitalOutputsRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """uint8 digital_out_0
uint8 digital_out_1
uint8 digital_out_2

"""
  __slots__ = ['digital_out_0','digital_out_1','digital_out_2']
  _slot_types = ['uint8','uint8','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       digital_out_0,digital_out_1,digital_out_2

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SetDigitalOutputsRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.digital_out_0 is None:
        self.digital_out_0 = 0
      if self.digital_out_1 is None:
        self.digital_out_1 = 0
      if self.digital_out_2 is None:
        self.digital_out_2 = 0
    else:
      self.digital_out_0 = 0
      self.digital_out_1 = 0
      self.digital_out_2 = 0

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
      buff.write(_struct_3B.pack(_x.digital_out_0, _x.digital_out_1, _x.digital_out_2))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 3
      (_x.digital_out_0, _x.digital_out_1, _x.digital_out_2,) = _struct_3B.unpack(str[start:end])
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
      buff.write(_struct_3B.pack(_x.digital_out_0, _x.digital_out_1, _x.digital_out_2))
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
      _x = self
      start = end
      end += 3
      (_x.digital_out_0, _x.digital_out_1, _x.digital_out_2,) = _struct_3B.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3B = struct.Struct("<3B")
"""autogenerated by genpy from turtlebot_node/SetDigitalOutputsResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class SetDigitalOutputsResponse(genpy.Message):
  _md5sum = "89bb254424e4cffedbf494e7b0ddbfea"
  _type = "turtlebot_node/SetDigitalOutputsResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """bool done

"""
  __slots__ = ['done']
  _slot_types = ['bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       done

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SetDigitalOutputsResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.done is None:
        self.done = False
    else:
      self.done = False

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
      buff.write(_struct_B.pack(self.done))
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
      end += 1
      (self.done,) = _struct_B.unpack(str[start:end])
      self.done = bool(self.done)
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
      buff.write(_struct_B.pack(self.done))
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
      end += 1
      (self.done,) = _struct_B.unpack(str[start:end])
      self.done = bool(self.done)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_B = struct.Struct("<B")
class SetDigitalOutputs(object):
  _type          = 'turtlebot_node/SetDigitalOutputs'
  _md5sum = '3e43640171aa67a865fe4990d6959f42'
  _request_class  = SetDigitalOutputsRequest
  _response_class = SetDigitalOutputsResponse
