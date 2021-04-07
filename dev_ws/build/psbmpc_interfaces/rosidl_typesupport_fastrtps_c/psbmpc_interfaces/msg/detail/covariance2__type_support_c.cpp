// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from psbmpc_interfaces:msg/Covariance2.idl
// generated code does not contain a copyright notice
#include "psbmpc_interfaces/msg/detail/covariance2__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "psbmpc_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "psbmpc_interfaces/msg/detail/covariance2__struct.h"
#include "psbmpc_interfaces/msg/detail/covariance2__functions.h"
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


// forward declare type support functions


using _Covariance2__ros_msg_type = psbmpc_interfaces__msg__Covariance2;

static bool _Covariance2__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Covariance2__ros_msg_type * ros_message = static_cast<const _Covariance2__ros_msg_type *>(untyped_ros_message);
  // Field name: var_x
  {
    cdr << ros_message->var_x;
  }

  // Field name: var_y
  {
    cdr << ros_message->var_y;
  }

  // Field name: cor_xy
  {
    cdr << ros_message->cor_xy;
  }

  return true;
}

static bool _Covariance2__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Covariance2__ros_msg_type * ros_message = static_cast<_Covariance2__ros_msg_type *>(untyped_ros_message);
  // Field name: var_x
  {
    cdr >> ros_message->var_x;
  }

  // Field name: var_y
  {
    cdr >> ros_message->var_y;
  }

  // Field name: cor_xy
  {
    cdr >> ros_message->cor_xy;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_psbmpc_interfaces
size_t get_serialized_size_psbmpc_interfaces__msg__Covariance2(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Covariance2__ros_msg_type * ros_message = static_cast<const _Covariance2__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name var_x
  {
    size_t item_size = sizeof(ros_message->var_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name var_y
  {
    size_t item_size = sizeof(ros_message->var_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cor_xy
  {
    size_t item_size = sizeof(ros_message->cor_xy);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Covariance2__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_psbmpc_interfaces__msg__Covariance2(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_psbmpc_interfaces
size_t max_serialized_size_psbmpc_interfaces__msg__Covariance2(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: var_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: var_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: cor_xy
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _Covariance2__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_psbmpc_interfaces__msg__Covariance2(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_Covariance2 = {
  "psbmpc_interfaces::msg",
  "Covariance2",
  _Covariance2__cdr_serialize,
  _Covariance2__cdr_deserialize,
  _Covariance2__get_serialized_size,
  _Covariance2__max_serialized_size
};

static rosidl_message_type_support_t _Covariance2__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Covariance2,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, psbmpc_interfaces, msg, Covariance2)() {
  return &_Covariance2__type_support;
}

#if defined(__cplusplus)
}
#endif
