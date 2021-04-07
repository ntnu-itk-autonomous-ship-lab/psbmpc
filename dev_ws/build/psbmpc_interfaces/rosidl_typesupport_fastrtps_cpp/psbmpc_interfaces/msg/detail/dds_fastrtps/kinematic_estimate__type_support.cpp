// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from psbmpc_interfaces:msg/KinematicEstimate.idl
// generated code does not contain a copyright notice
#include "psbmpc_interfaces/msg/detail/kinematic_estimate__rosidl_typesupport_fastrtps_cpp.hpp"
#include "psbmpc_interfaces/msg/detail/kinematic_estimate__struct.hpp"

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
bool cdr_serialize(
  const psbmpc_interfaces::msg::Vector2 &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  psbmpc_interfaces::msg::Vector2 &);
size_t get_serialized_size(
  const psbmpc_interfaces::msg::Vector2 &,
  size_t current_alignment);
size_t
max_serialized_size_Vector2(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace psbmpc_interfaces

namespace psbmpc_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const psbmpc_interfaces::msg::Vector2 &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  psbmpc_interfaces::msg::Vector2 &);
size_t get_serialized_size(
  const psbmpc_interfaces::msg::Vector2 &,
  size_t current_alignment);
size_t
max_serialized_size_Vector2(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace psbmpc_interfaces

namespace psbmpc_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const psbmpc_interfaces::msg::Covariance2 &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  psbmpc_interfaces::msg::Covariance2 &);
size_t get_serialized_size(
  const psbmpc_interfaces::msg::Covariance2 &,
  size_t current_alignment);
size_t
max_serialized_size_Covariance2(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace psbmpc_interfaces

namespace psbmpc_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const psbmpc_interfaces::msg::Covariance2 &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  psbmpc_interfaces::msg::Covariance2 &);
size_t get_serialized_size(
  const psbmpc_interfaces::msg::Covariance2 &,
  size_t current_alignment);
size_t
max_serialized_size_Covariance2(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace psbmpc_interfaces

namespace psbmpc_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const psbmpc_interfaces::msg::CrossCovariance2 &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  psbmpc_interfaces::msg::CrossCovariance2 &);
size_t get_serialized_size(
  const psbmpc_interfaces::msg::CrossCovariance2 &,
  size_t current_alignment);
size_t
max_serialized_size_CrossCovariance2(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace psbmpc_interfaces


namespace psbmpc_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_psbmpc_interfaces
cdr_serialize(
  const psbmpc_interfaces::msg::KinematicEstimate & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: pos_est
  psbmpc_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.pos_est,
    cdr);
  // Member: vel_est
  psbmpc_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.vel_est,
    cdr);
  // Member: pos_cov
  psbmpc_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.pos_cov,
    cdr);
  // Member: vel_cov
  psbmpc_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.vel_cov,
    cdr);
  // Member: pos_vel_corr
  psbmpc_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.pos_vel_corr,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_psbmpc_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  psbmpc_interfaces::msg::KinematicEstimate & ros_message)
{
  // Member: pos_est
  psbmpc_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.pos_est);

  // Member: vel_est
  psbmpc_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.vel_est);

  // Member: pos_cov
  psbmpc_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.pos_cov);

  // Member: vel_cov
  psbmpc_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.vel_cov);

  // Member: pos_vel_corr
  psbmpc_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.pos_vel_corr);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_psbmpc_interfaces
get_serialized_size(
  const psbmpc_interfaces::msg::KinematicEstimate & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: pos_est

  current_alignment +=
    psbmpc_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.pos_est, current_alignment);
  // Member: vel_est

  current_alignment +=
    psbmpc_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.vel_est, current_alignment);
  // Member: pos_cov

  current_alignment +=
    psbmpc_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.pos_cov, current_alignment);
  // Member: vel_cov

  current_alignment +=
    psbmpc_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.vel_cov, current_alignment);
  // Member: pos_vel_corr

  current_alignment +=
    psbmpc_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.pos_vel_corr, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_psbmpc_interfaces
max_serialized_size_KinematicEstimate(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: pos_est
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        psbmpc_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_Vector2(
        full_bounded, current_alignment);
    }
  }

  // Member: vel_est
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        psbmpc_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_Vector2(
        full_bounded, current_alignment);
    }
  }

  // Member: pos_cov
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        psbmpc_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_Covariance2(
        full_bounded, current_alignment);
    }
  }

  // Member: vel_cov
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        psbmpc_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_Covariance2(
        full_bounded, current_alignment);
    }
  }

  // Member: pos_vel_corr
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        psbmpc_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_CrossCovariance2(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static bool _KinematicEstimate__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const psbmpc_interfaces::msg::KinematicEstimate *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _KinematicEstimate__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<psbmpc_interfaces::msg::KinematicEstimate *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _KinematicEstimate__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const psbmpc_interfaces::msg::KinematicEstimate *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _KinematicEstimate__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_KinematicEstimate(full_bounded, 0);
}

static message_type_support_callbacks_t _KinematicEstimate__callbacks = {
  "psbmpc_interfaces::msg",
  "KinematicEstimate",
  _KinematicEstimate__cdr_serialize,
  _KinematicEstimate__cdr_deserialize,
  _KinematicEstimate__get_serialized_size,
  _KinematicEstimate__max_serialized_size
};

static rosidl_message_type_support_t _KinematicEstimate__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_KinematicEstimate__callbacks,
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
get_message_type_support_handle<psbmpc_interfaces::msg::KinematicEstimate>()
{
  return &psbmpc_interfaces::msg::typesupport_fastrtps_cpp::_KinematicEstimate__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, psbmpc_interfaces, msg, KinematicEstimate)() {
  return &psbmpc_interfaces::msg::typesupport_fastrtps_cpp::_KinematicEstimate__handle;
}

#ifdef __cplusplus
}
#endif
