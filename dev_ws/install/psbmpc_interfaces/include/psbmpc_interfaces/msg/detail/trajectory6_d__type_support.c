// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from psbmpc_interfaces:msg/Trajectory6D.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "psbmpc_interfaces/msg/detail/trajectory6_d__rosidl_typesupport_introspection_c.h"
#include "psbmpc_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "psbmpc_interfaces/msg/detail/trajectory6_d__functions.h"
#include "psbmpc_interfaces/msg/detail/trajectory6_d__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `predicted_trajectory`
#include "psbmpc_interfaces/msg/reference.h"
// Member `predicted_trajectory`
#include "psbmpc_interfaces/msg/detail/reference__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Trajectory6D__rosidl_typesupport_introspection_c__Trajectory6D_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  psbmpc_interfaces__msg__Trajectory6D__init(message_memory);
}

void Trajectory6D__rosidl_typesupport_introspection_c__Trajectory6D_fini_function(void * message_memory)
{
  psbmpc_interfaces__msg__Trajectory6D__fini(message_memory);
}

size_t Trajectory6D__rosidl_typesupport_introspection_c__size_function__Reference__predicted_trajectory(
  const void * untyped_member)
{
  const psbmpc_interfaces__msg__Reference__Sequence * member =
    (const psbmpc_interfaces__msg__Reference__Sequence *)(untyped_member);
  return member->size;
}

const void * Trajectory6D__rosidl_typesupport_introspection_c__get_const_function__Reference__predicted_trajectory(
  const void * untyped_member, size_t index)
{
  const psbmpc_interfaces__msg__Reference__Sequence * member =
    (const psbmpc_interfaces__msg__Reference__Sequence *)(untyped_member);
  return &member->data[index];
}

void * Trajectory6D__rosidl_typesupport_introspection_c__get_function__Reference__predicted_trajectory(
  void * untyped_member, size_t index)
{
  psbmpc_interfaces__msg__Reference__Sequence * member =
    (psbmpc_interfaces__msg__Reference__Sequence *)(untyped_member);
  return &member->data[index];
}

bool Trajectory6D__rosidl_typesupport_introspection_c__resize_function__Reference__predicted_trajectory(
  void * untyped_member, size_t size)
{
  psbmpc_interfaces__msg__Reference__Sequence * member =
    (psbmpc_interfaces__msg__Reference__Sequence *)(untyped_member);
  psbmpc_interfaces__msg__Reference__Sequence__fini(member);
  return psbmpc_interfaces__msg__Reference__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember Trajectory6D__rosidl_typesupport_introspection_c__Trajectory6D_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(psbmpc_interfaces__msg__Trajectory6D, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "predicted_trajectory",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(psbmpc_interfaces__msg__Trajectory6D, predicted_trajectory),  // bytes offset in struct
    NULL,  // default value
    Trajectory6D__rosidl_typesupport_introspection_c__size_function__Reference__predicted_trajectory,  // size() function pointer
    Trajectory6D__rosidl_typesupport_introspection_c__get_const_function__Reference__predicted_trajectory,  // get_const(index) function pointer
    Trajectory6D__rosidl_typesupport_introspection_c__get_function__Reference__predicted_trajectory,  // get(index) function pointer
    Trajectory6D__rosidl_typesupport_introspection_c__resize_function__Reference__predicted_trajectory  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Trajectory6D__rosidl_typesupport_introspection_c__Trajectory6D_message_members = {
  "psbmpc_interfaces__msg",  // message namespace
  "Trajectory6D",  // message name
  2,  // number of fields
  sizeof(psbmpc_interfaces__msg__Trajectory6D),
  Trajectory6D__rosidl_typesupport_introspection_c__Trajectory6D_message_member_array,  // message members
  Trajectory6D__rosidl_typesupport_introspection_c__Trajectory6D_init_function,  // function to initialize message memory (memory has to be allocated)
  Trajectory6D__rosidl_typesupport_introspection_c__Trajectory6D_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Trajectory6D__rosidl_typesupport_introspection_c__Trajectory6D_message_type_support_handle = {
  0,
  &Trajectory6D__rosidl_typesupport_introspection_c__Trajectory6D_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_psbmpc_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, psbmpc_interfaces, msg, Trajectory6D)() {
  Trajectory6D__rosidl_typesupport_introspection_c__Trajectory6D_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  Trajectory6D__rosidl_typesupport_introspection_c__Trajectory6D_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, psbmpc_interfaces, msg, Reference)();
  if (!Trajectory6D__rosidl_typesupport_introspection_c__Trajectory6D_message_type_support_handle.typesupport_identifier) {
    Trajectory6D__rosidl_typesupport_introspection_c__Trajectory6D_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Trajectory6D__rosidl_typesupport_introspection_c__Trajectory6D_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
