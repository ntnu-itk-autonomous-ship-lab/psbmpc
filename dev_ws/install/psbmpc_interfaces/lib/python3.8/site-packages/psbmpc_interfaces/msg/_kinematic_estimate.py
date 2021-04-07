# generated from rosidl_generator_py/resource/_idl.py.em
# with input from psbmpc_interfaces:msg/KinematicEstimate.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_KinematicEstimate(type):
    """Metaclass of message 'KinematicEstimate'."""

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
                'psbmpc_interfaces.msg.KinematicEstimate')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__kinematic_estimate
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__kinematic_estimate
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__kinematic_estimate
            cls._TYPE_SUPPORT = module.type_support_msg__msg__kinematic_estimate
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__kinematic_estimate

            from psbmpc_interfaces.msg import Covariance2
            if Covariance2.__class__._TYPE_SUPPORT is None:
                Covariance2.__class__.__import_type_support__()

            from psbmpc_interfaces.msg import CrossCovariance2
            if CrossCovariance2.__class__._TYPE_SUPPORT is None:
                CrossCovariance2.__class__.__import_type_support__()

            from psbmpc_interfaces.msg import Vector2
            if Vector2.__class__._TYPE_SUPPORT is None:
                Vector2.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class KinematicEstimate(metaclass=Metaclass_KinematicEstimate):
    """Message class 'KinematicEstimate'."""

    __slots__ = [
        '_pos_est',
        '_vel_est',
        '_pos_cov',
        '_vel_cov',
        '_pos_vel_corr',
    ]

    _fields_and_field_types = {
        'pos_est': 'psbmpc_interfaces/Vector2',
        'vel_est': 'psbmpc_interfaces/Vector2',
        'pos_cov': 'psbmpc_interfaces/Covariance2',
        'vel_cov': 'psbmpc_interfaces/Covariance2',
        'pos_vel_corr': 'psbmpc_interfaces/CrossCovariance2',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['psbmpc_interfaces', 'msg'], 'Vector2'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['psbmpc_interfaces', 'msg'], 'Vector2'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['psbmpc_interfaces', 'msg'], 'Covariance2'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['psbmpc_interfaces', 'msg'], 'Covariance2'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['psbmpc_interfaces', 'msg'], 'CrossCovariance2'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from psbmpc_interfaces.msg import Vector2
        self.pos_est = kwargs.get('pos_est', Vector2())
        from psbmpc_interfaces.msg import Vector2
        self.vel_est = kwargs.get('vel_est', Vector2())
        from psbmpc_interfaces.msg import Covariance2
        self.pos_cov = kwargs.get('pos_cov', Covariance2())
        from psbmpc_interfaces.msg import Covariance2
        self.vel_cov = kwargs.get('vel_cov', Covariance2())
        from psbmpc_interfaces.msg import CrossCovariance2
        self.pos_vel_corr = kwargs.get('pos_vel_corr', CrossCovariance2())

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
        if self.pos_est != other.pos_est:
            return False
        if self.vel_est != other.vel_est:
            return False
        if self.pos_cov != other.pos_cov:
            return False
        if self.vel_cov != other.vel_cov:
            return False
        if self.pos_vel_corr != other.pos_vel_corr:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def pos_est(self):
        """Message field 'pos_est'."""
        return self._pos_est

    @pos_est.setter
    def pos_est(self, value):
        if __debug__:
            from psbmpc_interfaces.msg import Vector2
            assert \
                isinstance(value, Vector2), \
                "The 'pos_est' field must be a sub message of type 'Vector2'"
        self._pos_est = value

    @property
    def vel_est(self):
        """Message field 'vel_est'."""
        return self._vel_est

    @vel_est.setter
    def vel_est(self, value):
        if __debug__:
            from psbmpc_interfaces.msg import Vector2
            assert \
                isinstance(value, Vector2), \
                "The 'vel_est' field must be a sub message of type 'Vector2'"
        self._vel_est = value

    @property
    def pos_cov(self):
        """Message field 'pos_cov'."""
        return self._pos_cov

    @pos_cov.setter
    def pos_cov(self, value):
        if __debug__:
            from psbmpc_interfaces.msg import Covariance2
            assert \
                isinstance(value, Covariance2), \
                "The 'pos_cov' field must be a sub message of type 'Covariance2'"
        self._pos_cov = value

    @property
    def vel_cov(self):
        """Message field 'vel_cov'."""
        return self._vel_cov

    @vel_cov.setter
    def vel_cov(self, value):
        if __debug__:
            from psbmpc_interfaces.msg import Covariance2
            assert \
                isinstance(value, Covariance2), \
                "The 'vel_cov' field must be a sub message of type 'Covariance2'"
        self._vel_cov = value

    @property
    def pos_vel_corr(self):
        """Message field 'pos_vel_corr'."""
        return self._pos_vel_corr

    @pos_vel_corr.setter
    def pos_vel_corr(self, value):
        if __debug__:
            from psbmpc_interfaces.msg import CrossCovariance2
            assert \
                isinstance(value, CrossCovariance2), \
                "The 'pos_vel_corr' field must be a sub message of type 'CrossCovariance2'"
        self._pos_vel_corr = value
