// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from psbmpc_interfaces:msg/KinematicEstimate.idl
// generated code does not contain a copyright notice
#include "psbmpc_interfaces/msg/detail/kinematic_estimate__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "psbmpc_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "psbmpc_interfaces/msg/detail/kinematic_estimate__struct.h"
#include "psbmpc_interfaces/msg/detail/kinematic_estimate__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "psbmpc_interfaces/msg/detail/covariance2__functions.h"  // pos_cov, vel_cov
#include "psbmpc_interfaces/msg/detail/cross_covariance2__functions.h"  // pos_vel_corr
#include "psbmpc_interfaces/msg/detail/vector2__functions.h"  // pos_est, vel_est

// forward declare type support functions
size_t get_serialized_size_psbmpc_interfaces__msg__Covariance2(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_psbmpc_interfaces__msg__Covariance2(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, psbmpc_interfaces, msg, Covariance2)();
size_t get_serialized_size_psbmpc_interfaces__msg__CrossCovariance2(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_psbmpc_interfaces__msg__CrossCovariance2(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, psbmpc_interfaces, msg, CrossCovariance2)();
size_t get_serialized_size_psbmpc_interfaces__msg__Vector2(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_psbmpc_interfaces__msg__Vector2(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, psbmpc_interfaces, msg, Vector2)();


using _KinematicEstimate__ros_msg_type = psbmpc_interfaces__msg__KinematicEstimate;

static bool _KinematicEstimate__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _KinematicEstimate__ros_msg_type * ros_message = static_cast<const _KinematicEstimate__ros_msg_type *>(untyped_ros_message);
  // Field name: pos_est
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, psbmpc_interfaces, msg, Vector2
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->pos_est, cdr))
    {
      return false;
    }
  }

  // Field name: vel_est
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, psbmpc_interfaces, msg, Vector2
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->vel_est, cdr))
    {
      return false;
    }
  }

  // Field name: pos_cov
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, psbmpc_interfaces, msg, Covariance2
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->pos_cov, cdr))
    {
      return false;
    }
  }

  // Field name: vel_cov
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, psbmpc_interfaces, msg, Covariance2
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->vel_cov, cdr))
    {
      return false;
    }
  }

  // Field name: pos_vel_corr
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, psbmpc_interfaces, msg, CrossCovariance2
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->pos_vel_corr, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _KinematicEstimate__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _KinematicEstimate__ros_msg_type * ros_message = static_cast<_KinematicEstimate__ros_msg_type *>(untyped_ros_message);
  // Field name: pos_est
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, psbmpc_interfaces, msg, Vector2
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->pos_est))
    {
      return false;
    }
  }

  // Field name: vel_est
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, psbmpc_interfaces, msg, Vector2
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->vel_est))
    {
      return false;
    }
  }

  // Field name: pos_cov
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, psbmpc_interfaces, msg, Covariance2
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->pos_cov))
    {
      return false;
    }
  }

  // Field name: vel_cov
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, psbmpc_interfaces, msg, Covariance2
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->vel_cov))
    {
      return false;
    }
  }

  // Field name: pos_vel_corr
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, psbmpc_interfaces, msg, CrossCovariance2
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->pos_vel_corr))
    {
      return false;
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_psbmpc_interfaces
size_t get_serialized_size_psbmpc_interfaces__msg__KinematicEstimate(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _KinematicEstimate__ros_msg_type * ros_message = static_cast<const _KinematicEstimate__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name pos_est

  current_alignment += get_serialized_size_psbmpc_interfaces__msg__Vector2(
    &(ros_message->pos_est), current_alignment);
  // field.name vel_est

  current_alignment += get_serialized_size_psbmpc_interfaces__msg__Vector2(
    &(ros_message->vel_est), current_alignment);
  // field.name pos_cov

  current_alignment += get_serialized_size_psbmpc_interfaces__msg__Covariance2(
    &(ros_message->pos_cov), current_alignment);
  // field.name vel_cov

  current_alignment += get_serialized_size_psbmpc_interfaces__msg__Covariance2(
    &(ros_message->vel_cov), current_alignment);
  // field.name pos_vel_corr

  current_alignment += get_serialized_size_psbmpc_interfaces__msg__CrossCovariance2(
    &(ros_message->pos_vel_corr), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _KinematicEstimate__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_psbmpc_interfaces__msg__KinematicEstimate(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_psbmpc_interfaces
size_t max_serialized_size_psbmpc_interfaces__msg__KinematicEstimate(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: pos_est
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_psbmpc_interfaces__msg__Vector2(
        full_bounded, current_alignment);
    }
  }
  // member: vel_est
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_psbmpc_interfaces__msg__Vector2(
        full_bounded, current_alignment);
    }
  }
  // member: pos_cov
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_psbmpc_interfaces__msg__Covariance2(
        full_bounded, current_alignment);
    }
  }
  // member: vel_cov
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_psbmpc_interfaces__msg__Covariance2(
        full_bounded, current_alignment);
    }
  }
  // member: pos_vel_corr
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_psbmpc_interfaces__msg__CrossCovariance2(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _KinematicEstimate__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_psbmpc_interfaces__msg__KinematicEstimate(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_KinematicEstimate = {
  "psbmpc_interfaces::msg",
  "KinematicEstimate",
  _KinematicEstimate__cdr_serialize,
  _KinematicEstimate__cdr_deserialize,
  _KinematicEstimate__get_serialized_size,
  _KinematicEstimate__max_serialized_size
};

static rosidl_message_type_support_t _KinematicEstimate__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_KinematicEstimate,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, psbmpc_interfaces, msg, KinematicEstimate)() {
  return &_KinematicEstimate__type_support;
}

#if defined(__cplusplus)
}
#endif
