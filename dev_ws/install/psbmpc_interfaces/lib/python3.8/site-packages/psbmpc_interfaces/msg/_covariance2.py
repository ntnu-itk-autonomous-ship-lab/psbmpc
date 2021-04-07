# generated from rosidl_generator_py/resource/_idl.py.em
# with input from psbmpc_interfaces:msg/Covariance2.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Covariance2(type):
    """Metaclass of message 'Covariance2'."""

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
                'psbmpc_interfaces.msg.Covariance2')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__covariance2
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__covariance2
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__covariance2
            cls._TYPE_SUPPORT = module.type_support_msg__msg__covariance2
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__covariance2

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Covariance2(metaclass=Metaclass_Covariance2):
    """Message class 'Covariance2'."""

    __slots__ = [
        '_var_x',
        '_var_y',
        '_cor_xy',
    ]

    _fields_and_field_types = {
        'var_x': 'double',
        'var_y': 'double',
        'cor_xy': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.var_x = kwargs.get('var_x', float())
        self.var_y = kwargs.get('var_y', float())
        self.cor_xy = kwargs.get('cor_xy', float())

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
        if self.var_x != other.var_x:
            return False
        if self.var_y != other.var_y:
            return False
        if self.cor_xy != other.cor_xy:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def var_x(self):
        """Message field 'var_x'."""
        return self._var_x

    @var_x.setter
    def var_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'var_x' field must be of type 'float'"
        self._var_x = value

    @property
    def var_y(self):
        """Message field 'var_y'."""
        return self._var_y

    @var_y.setter
    def var_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'var_y' field must be of type 'float'"
        self._var_y = value

    @property
    def cor_xy(self):
        """Message field 'cor_xy'."""
        return self._cor_xy

    @cor_xy.setter
    def cor_xy(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'cor_xy' field must be of type 'float'"
        self._cor_xy = value
