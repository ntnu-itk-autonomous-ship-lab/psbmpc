// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from psbmpc_interfaces:msg/Covariance2.idl
// generated code does not contain a copyright notice
#include "psbmpc_interfaces/msg/detail/covariance2__rosidl_typesupport_fastrtps_cpp.hpp"
#include "psbmpc_interfaces/msg/detail/covariance2__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace psbmpc_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_psbmpc_interfaces
cdr_serialize(
  const psbmpc_interfaces::msg::Covariance2 & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: var_x
  cdr << ros_message.var_x;
  // Member: var_y
  cdr << ros_message.var_y;
  // Member: cor_xy
  cdr << ros_message.cor_xy;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_psbmpc_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  psbmpc_interfaces::msg::Covariance2 & ros_message)
{
  // Member: var_x
  cdr >> ros_message.var_x;

  // Member: var_y
  cdr >> ros_message.var_y;

  // Member: cor_xy
  cdr >> ros_message.cor_xy;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_psbmpc_interfaces
get_serialized_size(
  const psbmpc_interfaces::msg::Covariance2 & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: var_x
  {
    size_t item_size = sizeof(ros_message.var_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: var_y
  {
    size_t item_size = sizeof(ros_message.var_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: cor_xy
  {
    size_t item_size = sizeof(ros_message.cor_xy);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_psbmpc_interfaces
max_serialized_size_Covariance2(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: var_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: var_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: cor_xy
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  return current_alignment - initial_alignment;
}

static bool _Covariance2__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const psbmpc_interfaces::msg::Covariance2 *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Covariance2__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<psbmpc_interfaces::msg::Covariance2 *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Covariance2__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const psbmpc_interfaces::msg::Covariance2 *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Covariance2__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_Covariance2(full_bounded, 0);
}

static message_type_support_callbacks_t _Covariance2__callbacks = {
  "psbmpc_interfaces::msg",
  "Covariance2",
  _Covariance2__cdr_serialize,
  _Covariance2__cdr_deserialize,
  _Covariance2__get_serialized_size,
  _Covariance2__max_serialized_size
};

static rosidl_message_type_support_t _Covariance2__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Covariance2__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace psbmpc_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_psbmpc_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<psbmpc_interfaces::msg::Covariance2>()
{
  return &psbmpc_interfaces::msg::typesupport_fastrtps_cpp::_Covariance2__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, psbmpc_interfaces, msg, Covariance2)() {
  return &psbmpc_interfaces::msg::typesupport_fastrtps_cpp::_Covariance2__handle;
}

#ifdef __cplusplus
}
#endif
