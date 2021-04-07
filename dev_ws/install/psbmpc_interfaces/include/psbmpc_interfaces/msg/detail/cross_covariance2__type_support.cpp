// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from psbmpc_interfaces:msg/CrossCovariance2.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "psbmpc_interfaces/msg/detail/cross_covariance2__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace psbmpc_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void CrossCovariance2_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) psbmpc_interfaces::msg::CrossCovariance2(_init);
}

void CrossCovariance2_fini_function(void * message_memory)
{
  auto typed_message = static_cast<psbmpc_interfaces::msg::CrossCovariance2 *>(message_memory);
  typed_message->~CrossCovariance2();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CrossCovariance2_message_member_array[4] = {
  {
    "cor_px_vx",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(psbmpc_interfaces::msg::CrossCovariance2, cor_px_vx),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "cor_px_vy",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(psbmpc_interfaces::msg::CrossCovariance2, cor_px_vy),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "cor_py_vx",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(psbmpc_interfaces::msg::CrossCovariance2, cor_py_vx),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "cor_py_vy",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(psbmpc_interfaces::msg::CrossCovariance2, cor_py_vy),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CrossCovariance2_message_members = {
  "psbmpc_interfaces::msg",  // message namespace
  "CrossCovariance2",  // message name
  4,  // number of fields
  sizeof(psbmpc_interfaces::msg::CrossCovariance2),
  CrossCovariance2_message_member_array,  // message members
  CrossCovariance2_init_function,  // function to initialize message memory (memory has to be allocated)
  CrossCovariance2_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CrossCovariance2_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CrossCovariance2_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace psbmpc_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<psbmpc_interfaces::msg::CrossCovariance2>()
{
  return &::psbmpc_interfaces::msg::rosidl_typesupport_introspection_cpp::CrossCovariance2_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, psbmpc_interfaces, msg, CrossCovariance2)() {
  return &::psbmpc_interfaces::msg::rosidl_typesupport_introspection_cpp::CrossCovariance2_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
