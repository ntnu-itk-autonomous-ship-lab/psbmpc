# generated from rosidl_generator_py/resource/_idl.py.em
# with input from psbmpc_interfaces:msg/CrossCovariance2.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CrossCovariance2(type):
    """Metaclass of message 'CrossCovariance2'."""

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
            module = import_type_support('psbmpc_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'psbmpc_interfaces.msg.CrossCovariance2')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__cross_covariance2
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__cross_covariance2
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__cross_covariance2
            cls._TYPE_SUPPORT = module.type_support_msg__msg__cross_covariance2
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__cross_covariance2

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CrossCovariance2(metaclass=Metaclass_CrossCovariance2):
    """Message class 'CrossCovariance2'."""

    __slots__ = [
        '_cor_px_vx',
        '_cor_px_vy',
        '_cor_py_vx',
        '_cor_py_vy',
    ]

    _fields_and_field_types = {
        'cor_px_vx': 'double',
        'cor_px_vy': 'double',
        'cor_py_vx': 'double',
        'cor_py_vy': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.cor_px_vx = kwargs.get('cor_px_vx', float())
        self.cor_px_vy = kwargs.get('cor_px_vy', float())
        self.cor_py_vx = kwargs.get('cor_py_vx', float())
        self.cor_py_vy = kwargs.get('cor_py_vy', float())

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
        if self.cor_px_vx != other.cor_px_vx:
            return False
        if self.cor_px_vy != other.cor_px_vy:
            return False
        if self.cor_py_vx != other.cor_py_vx:
            return False
        if self.cor_py_vy != other.cor_py_vy:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def cor_px_vx(self):
        """Message field 'cor_px_vx'."""
        return self._cor_px_vx

    @cor_px_vx.setter
    def cor_px_vx(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cor_px_vx' field must be of type 'float'"
        self._cor_px_vx = value

    @property
    def cor_px_vy(self):
        """Message field 'cor_px_vy'."""
        return self._cor_px_vy

    @cor_px_vy.setter
    def cor_px_vy(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cor_px_vy' field must be of type 'float'"
        self._cor_px_vy = value

    @property
    def cor_py_vx(self):
        """Message field 'cor_py_vx'."""
        return self._cor_py_vx

    @cor_py_vx.setter
    def cor_py_vx(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cor_py_vx' field must be of type 'float'"
        self._cor_py_vx = value

    @property
    def cor_py_vy(self):
        """Message field 'cor_py_vy'."""
        return self._cor_py_vy

    @cor_py_vy.setter
    def cor_py_vy(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cor_py_vy' field must be of type 'float'"
        self._cor_py_vy = value
