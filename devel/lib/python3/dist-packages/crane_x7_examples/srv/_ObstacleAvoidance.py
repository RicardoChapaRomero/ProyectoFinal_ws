# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from crane_x7_examples/ObstacleAvoidanceRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import std_msgs.msg

class ObstacleAvoidanceRequest(genpy.Message):
  _md5sum = "1f107495b21e60af9baa011dd99ed578"
  _type = "crane_x7_examples/ObstacleAvoidanceRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# target pose for the arm
geometry_msgs/Pose          start_pose
geometry_msgs/Pose          goal_pose

# the obstacle shape is BOX
bool                        obstacle_enable
geometry_msgs/Vector3       obstacle_size
geometry_msgs/PoseStamped   obstacle_pose_stamped
string                      obstacle_name

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
"""
  __slots__ = ['start_pose','goal_pose','obstacle_enable','obstacle_size','obstacle_pose_stamped','obstacle_name']
  _slot_types = ['geometry_msgs/Pose','geometry_msgs/Pose','bool','geometry_msgs/Vector3','geometry_msgs/PoseStamped','string']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       start_pose,goal_pose,obstacle_enable,obstacle_size,obstacle_pose_stamped,obstacle_name

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ObstacleAvoidanceRequest, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.start_pose is None:
        self.start_pose = geometry_msgs.msg.Pose()
      if self.goal_pose is None:
        self.goal_pose = geometry_msgs.msg.Pose()
      if self.obstacle_enable is None:
        self.obstacle_enable = False
      if self.obstacle_size is None:
        self.obstacle_size = geometry_msgs.msg.Vector3()
      if self.obstacle_pose_stamped is None:
        self.obstacle_pose_stamped = geometry_msgs.msg.PoseStamped()
      if self.obstacle_name is None:
        self.obstacle_name = ''
    else:
      self.start_pose = geometry_msgs.msg.Pose()
      self.goal_pose = geometry_msgs.msg.Pose()
      self.obstacle_enable = False
      self.obstacle_size = geometry_msgs.msg.Vector3()
      self.obstacle_pose_stamped = geometry_msgs.msg.PoseStamped()
      self.obstacle_name = ''

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
      buff.write(_get_struct_14dB3d3I().pack(_x.start_pose.position.x, _x.start_pose.position.y, _x.start_pose.position.z, _x.start_pose.orientation.x, _x.start_pose.orientation.y, _x.start_pose.orientation.z, _x.start_pose.orientation.w, _x.goal_pose.position.x, _x.goal_pose.position.y, _x.goal_pose.position.z, _x.goal_pose.orientation.x, _x.goal_pose.orientation.y, _x.goal_pose.orientation.z, _x.goal_pose.orientation.w, _x.obstacle_enable, _x.obstacle_size.x, _x.obstacle_size.y, _x.obstacle_size.z, _x.obstacle_pose_stamped.header.seq, _x.obstacle_pose_stamped.header.stamp.secs, _x.obstacle_pose_stamped.header.stamp.nsecs))
      _x = self.obstacle_pose_stamped.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_7d().pack(_x.obstacle_pose_stamped.pose.position.x, _x.obstacle_pose_stamped.pose.position.y, _x.obstacle_pose_stamped.pose.position.z, _x.obstacle_pose_stamped.pose.orientation.x, _x.obstacle_pose_stamped.pose.orientation.y, _x.obstacle_pose_stamped.pose.orientation.z, _x.obstacle_pose_stamped.pose.orientation.w))
      _x = self.obstacle_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.start_pose is None:
        self.start_pose = geometry_msgs.msg.Pose()
      if self.goal_pose is None:
        self.goal_pose = geometry_msgs.msg.Pose()
      if self.obstacle_size is None:
        self.obstacle_size = geometry_msgs.msg.Vector3()
      if self.obstacle_pose_stamped is None:
        self.obstacle_pose_stamped = geometry_msgs.msg.PoseStamped()
      end = 0
      _x = self
      start = end
      end += 149
      (_x.start_pose.position.x, _x.start_pose.position.y, _x.start_pose.position.z, _x.start_pose.orientation.x, _x.start_pose.orientation.y, _x.start_pose.orientation.z, _x.start_pose.orientation.w, _x.goal_pose.position.x, _x.goal_pose.position.y, _x.goal_pose.position.z, _x.goal_pose.orientation.x, _x.goal_pose.orientation.y, _x.goal_pose.orientation.z, _x.goal_pose.orientation.w, _x.obstacle_enable, _x.obstacle_size.x, _x.obstacle_size.y, _x.obstacle_size.z, _x.obstacle_pose_stamped.header.seq, _x.obstacle_pose_stamped.header.stamp.secs, _x.obstacle_pose_stamped.header.stamp.nsecs,) = _get_struct_14dB3d3I().unpack(str[start:end])
      self.obstacle_enable = bool(self.obstacle_enable)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.obstacle_pose_stamped.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.obstacle_pose_stamped.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 56
      (_x.obstacle_pose_stamped.pose.position.x, _x.obstacle_pose_stamped.pose.position.y, _x.obstacle_pose_stamped.pose.position.z, _x.obstacle_pose_stamped.pose.orientation.x, _x.obstacle_pose_stamped.pose.orientation.y, _x.obstacle_pose_stamped.pose.orientation.z, _x.obstacle_pose_stamped.pose.orientation.w,) = _get_struct_7d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.obstacle_name = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.obstacle_name = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_14dB3d3I().pack(_x.start_pose.position.x, _x.start_pose.position.y, _x.start_pose.position.z, _x.start_pose.orientation.x, _x.start_pose.orientation.y, _x.start_pose.orientation.z, _x.start_pose.orientation.w, _x.goal_pose.position.x, _x.goal_pose.position.y, _x.goal_pose.position.z, _x.goal_pose.orientation.x, _x.goal_pose.orientation.y, _x.goal_pose.orientation.z, _x.goal_pose.orientation.w, _x.obstacle_enable, _x.obstacle_size.x, _x.obstacle_size.y, _x.obstacle_size.z, _x.obstacle_pose_stamped.header.seq, _x.obstacle_pose_stamped.header.stamp.secs, _x.obstacle_pose_stamped.header.stamp.nsecs))
      _x = self.obstacle_pose_stamped.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_7d().pack(_x.obstacle_pose_stamped.pose.position.x, _x.obstacle_pose_stamped.pose.position.y, _x.obstacle_pose_stamped.pose.position.z, _x.obstacle_pose_stamped.pose.orientation.x, _x.obstacle_pose_stamped.pose.orientation.y, _x.obstacle_pose_stamped.pose.orientation.z, _x.obstacle_pose_stamped.pose.orientation.w))
      _x = self.obstacle_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.start_pose is None:
        self.start_pose = geometry_msgs.msg.Pose()
      if self.goal_pose is None:
        self.goal_pose = geometry_msgs.msg.Pose()
      if self.obstacle_size is None:
        self.obstacle_size = geometry_msgs.msg.Vector3()
      if self.obstacle_pose_stamped is None:
        self.obstacle_pose_stamped = geometry_msgs.msg.PoseStamped()
      end = 0
      _x = self
      start = end
      end += 149
      (_x.start_pose.position.x, _x.start_pose.position.y, _x.start_pose.position.z, _x.start_pose.orientation.x, _x.start_pose.orientation.y, _x.start_pose.orientation.z, _x.start_pose.orientation.w, _x.goal_pose.position.x, _x.goal_pose.position.y, _x.goal_pose.position.z, _x.goal_pose.orientation.x, _x.goal_pose.orientation.y, _x.goal_pose.orientation.z, _x.goal_pose.orientation.w, _x.obstacle_enable, _x.obstacle_size.x, _x.obstacle_size.y, _x.obstacle_size.z, _x.obstacle_pose_stamped.header.seq, _x.obstacle_pose_stamped.header.stamp.secs, _x.obstacle_pose_stamped.header.stamp.nsecs,) = _get_struct_14dB3d3I().unpack(str[start:end])
      self.obstacle_enable = bool(self.obstacle_enable)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.obstacle_pose_stamped.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.obstacle_pose_stamped.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 56
      (_x.obstacle_pose_stamped.pose.position.x, _x.obstacle_pose_stamped.pose.position.y, _x.obstacle_pose_stamped.pose.position.z, _x.obstacle_pose_stamped.pose.orientation.x, _x.obstacle_pose_stamped.pose.orientation.y, _x.obstacle_pose_stamped.pose.orientation.z, _x.obstacle_pose_stamped.pose.orientation.w,) = _get_struct_7d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.obstacle_name = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.obstacle_name = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_14dB3d3I = None
def _get_struct_14dB3d3I():
    global _struct_14dB3d3I
    if _struct_14dB3d3I is None:
        _struct_14dB3d3I = struct.Struct("<14dB3d3I")
    return _struct_14dB3d3I
_struct_7d = None
def _get_struct_7d():
    global _struct_7d
    if _struct_7d is None:
        _struct_7d = struct.Struct("<7d")
    return _struct_7d
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from crane_x7_examples/ObstacleAvoidanceResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class ObstacleAvoidanceResponse(genpy.Message):
  _md5sum = "eb13ac1f1354ccecb7941ee8fa2192e8"
  _type = "crane_x7_examples/ObstacleAvoidanceResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """bool                        result

"""
  __slots__ = ['result']
  _slot_types = ['bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       result

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ObstacleAvoidanceResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.result is None:
        self.result = False
    else:
      self.result = False

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
      _x = self.result
      buff.write(_get_struct_B().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 1
      (self.result,) = _get_struct_B().unpack(str[start:end])
      self.result = bool(self.result)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self.result
      buff.write(_get_struct_B().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 1
      (self.result,) = _get_struct_B().unpack(str[start:end])
      self.result = bool(self.result)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_B = None
def _get_struct_B():
    global _struct_B
    if _struct_B is None:
        _struct_B = struct.Struct("<B")
    return _struct_B
class ObstacleAvoidance(object):
  _type          = 'crane_x7_examples/ObstacleAvoidance'
  _md5sum = '72c38d661db946d478d0ae7732add5d5'
  _request_class  = ObstacleAvoidanceRequest
  _response_class = ObstacleAvoidanceResponse