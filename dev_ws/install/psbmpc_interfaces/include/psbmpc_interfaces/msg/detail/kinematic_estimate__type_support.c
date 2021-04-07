// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from psbmpc_interfaces:msg/KinematicEstimate.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "psbmpc_interfaces/msg/detail/kinematic_estimate__rosidl_typesupport_introspection_c.h"
#include "psbmpc_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "psbmpc_interfaces/msg/detail/kinematic_estimate__functions.h"
#include "psbmpc_interfaces/msg/detail/kinematic_estimate__struct.h"


// Include directives for member types
// Member `pos_est`
// Member `vel_est`
#include "psbmpc_interfaces/msg/vector2.h"
// Member `pos_est`
// Member `vel_est`
#include "psbmpc_interfaces/msg/detail/vector2__rosidl_typesupport_introspection_c.h"
// Member `pos_cov`
// Member `vel_cov`
#include "psbmpc_interfaces/msg/covariance2.h"
// Member `pos_cov`
// Member `vel_cov`
#include "psbmpc_interfaces/msg/detail/covariance2__rosidl_typesupport_introspection_c.h"
// Member `pos_vel_corr`
#include "psbmpc_interfaces/msg/cross_covariance2.h"
// Member `pos_vel_corr`
#include "psbmpc_interfaces/msg/detail/cross_covariance2__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void KinematicEstimate__rosidl_typesupport_introspection_c__KinematicEstimate_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  psbmpc_interfaces__msg__KinematicEstimate__init(message_memory);
}

void KinematicEstimate__rosidl_typesupport_introspection_c__KinematicEstimate_fini_function(void * message_memory)
{
  psbmpc_interfaces__msg__KinematicEstimate__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember KinematicEstimate__rosidl_typesupport_introspection_c__KinematicEstimate_message_member_array[5] = {
  {
    "pos_est",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(psbmpc_interfaces__msg__KinematicEstimate, pos_est),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "vel_est",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(psbmpc_interfaces__msg__KinematicEstimate, vel_est),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pos_cov",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(psbmpc_interfaces__msg__KinematicEstimate, pos_cov),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "vel_cov",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(psbmpc_interfaces__msg__KinematicEstimate, vel_cov),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pos_vel_corr",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(psbmpc_interfaces__msg__KinematicEstimate, pos_vel_corr),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers KinematicEstimate__rosidl_typesupport_introspection_c__KinematicEstimate_message_members = {
  "psbmpc_interfaces__msg",  // message namespace
  "KinematicEstimate",  // message name
  5,  // number of fields
  sizeof(psbmpc_interfaces__msg__KinematicEstimate),
  KinematicEstimate__rosidl_typesupport_introspection_c__KinematicEstimate_message_member_array,  // message members
  KinematicEstimate__rosidl_typesupport_introspection_c__KinematicEstimate_init_function,  // function to initialize message memory (memory has to be allocated)
  KinematicEstimate__rosidl_typesupport_introspection_c__KinematicEstimate_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t KinematicEstimate__rosidl_typesupport_introspection_c__KinematicEstimate_message_type_support_handle = {
  0,
  &KinematicEstimate__rosidl_typesupport_introspection_c__KinematicEstimate_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_psbmpc_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, psbmpc_interfaces, msg, KinematicEstimate)() {
  KinematicEstimate__rosidl_typesupport_introspection_c__KinematicEstimate_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, psbmpc_interfaces, msg, Vector2)();
  KinematicEstimate__rosidl_typesupport_introspection_c__KinematicEstimate_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, psbmpc_interfaces, msg, Vector2)();
  KinematicEstimate__rosidl_typesupport_introspection_c__KinematicEstimate_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, psbmpc_interfaces, msg, Covariance2)();
  KinematicEstimate__rosidl_typesupport_introspection_c__KinematicEstimate_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, psbmpc_interfaces, msg, Covariance2)();
  KinematicEstimate__rosidl_typesupport_introspection_c__KinematicEstimate_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, psbmpc_interfaces, msg, CrossCovariance2)();
  if (!KinematicEstimate__rosidl_typesupport_introspection_c__KinematicEstimate_message_type_support_handle.typesupport_identifier) {
    KinematicEstimate__rosidl_typesupport_introspection_c__KinematicEstimate_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &KinematicEstimate__rosidl_typesupport_introspection_c__KinematicEstimate_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
