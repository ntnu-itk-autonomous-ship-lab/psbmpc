// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from psbmpc_interfaces:msg/CrossCovariance2.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "psbmpc_interfaces/msg/detail/cross_covariance2__rosidl_typesupport_introspection_c.h"
#include "psbmpc_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "psbmpc_interfaces/msg/detail/cross_covariance2__functions.h"
#include "psbmpc_interfaces/msg/detail/cross_covariance2__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void CrossCovariance2__rosidl_typesupport_introspection_c__CrossCovariance2_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  psbmpc_interfaces__msg__CrossCovariance2__init(message_memory);
}

void CrossCovariance2__rosidl_typesupport_introspection_c__CrossCovariance2_fini_function(void * message_memory)
{
  psbmpc_interfaces__msg__CrossCovariance2__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember CrossCovariance2__rosidl_typesupport_introspection_c__CrossCovariance2_message_member_array[4] = {
  {
    "cor_px_vx",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(psbmpc_interfaces__msg__CrossCovariance2, cor_px_vx),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cor_px_vy",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(psbmpc_interfaces__msg__CrossCovariance2, cor_px_vy),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cor_py_vx",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(psbmpc_interfaces__msg__CrossCovariance2, cor_py_vx),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cor_py_vy",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(psbmpc_interfaces__msg__CrossCovariance2, cor_py_vy),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers CrossCovariance2__rosidl_typesupport_introspection_c__CrossCovariance2_message_members = {
  "psbmpc_interfaces__msg",  // message namespace
  "CrossCovariance2",  // message name
  4,  // number of fields
  sizeof(psbmpc_interfaces__msg__CrossCovariance2),
  CrossCovariance2__rosidl_typesupport_introspection_c__CrossCovariance2_message_member_array,  // message members
  CrossCovariance2__rosidl_typesupport_introspection_c__CrossCovariance2_init_function,  // function to initialize message memory (memory has to be allocated)
  CrossCovariance2__rosidl_typesupport_introspection_c__CrossCovariance2_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t CrossCovariance2__rosidl_typesupport_introspection_c__CrossCovariance2_message_type_support_handle = {
  0,
  &CrossCovariance2__rosidl_typesupport_introspection_c__CrossCovariance2_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_psbmpc_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, psbmpc_interfaces, msg, CrossCovariance2)() {
  if (!CrossCovariance2__rosidl_typesupport_introspection_c__CrossCovariance2_message_type_support_handle.typesupport_identifier) {
    CrossCovariance2__rosidl_typesupport_introspection_c__CrossCovariance2_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &CrossCovariance2__rosidl_typesupport_introspection_c__CrossCovariance2_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
