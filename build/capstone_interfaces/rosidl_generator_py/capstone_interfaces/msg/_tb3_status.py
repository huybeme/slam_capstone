# generated from rosidl_generator_py/resource/_idl.py.em
# with input from capstone_interfaces:msg/TB3Status.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'lidar_data'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TB3Status(type):
    """Metaclass of message 'TB3Status'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('capstone_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'capstone_interfaces.msg.TB3Status')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__tb3_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__tb3_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__tb3_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__tb3_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__tb3_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'HIT_WALL__DEFAULT': False,
        }

    @property
    def HIT_WALL__DEFAULT(cls):
        """Return default value for message field 'hit_wall'."""
        return False


class TB3Status(metaclass=Metaclass_TB3Status):
    """Message class 'TB3Status'."""

    __slots__ = [
        '_lidar_data',
        '_hit_wall',
    ]

    _fields_and_field_types = {
        'lidar_data': 'float[8]',
        'hit_wall': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 8),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        if 'lidar_data' not in kwargs:
            self.lidar_data = numpy.zeros(8, dtype=numpy.float32)
        else:
            self.lidar_data = numpy.array(kwargs.get('lidar_data'), dtype=numpy.float32)
            assert self.lidar_data.shape == (8, )
        self.hit_wall = kwargs.get(
            'hit_wall', TB3Status.HIT_WALL__DEFAULT)

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if all(self.lidar_data != other.lidar_data):
            return False
        if self.hit_wall != other.hit_wall:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def lidar_data(self):
        """Message field 'lidar_data'."""
        return self._lidar_data

    @lidar_data.setter
    def lidar_data(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'lidar_data' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 8, \
                "The 'lidar_data' numpy.ndarray() must have a size of 8"
            self._lidar_data = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 8 and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'lidar_data' field must be a set or sequence with length 8 and each value of type 'float'"
        self._lidar_data = numpy.array(value, dtype=numpy.float32)

    @property
    def hit_wall(self):
        """Message field 'hit_wall'."""
        return self._hit_wall

    @hit_wall.setter
    def hit_wall(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'hit_wall' field must be of type 'bool'"
        self._hit_wall = value
