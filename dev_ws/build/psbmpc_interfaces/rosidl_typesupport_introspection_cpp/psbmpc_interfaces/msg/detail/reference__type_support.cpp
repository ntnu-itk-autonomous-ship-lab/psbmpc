// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from psbmpc_interfaces:msg/Reference.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "psbmpc_interfaces/msg/detail/reference__struct.hpp"
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

void Reference_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) psbmpc_interfaces::msg::Reference(_init);
}

void Reference_fini_function(void * message_memory)
{
  auto typed_message = static_cast<psbmpc_interfaces::msg::Reference *>(message_memory);
  typed_message->~Reference();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Reference_message_member_array[2] = {
  {
    "configuration",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Vector3>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(psbmpc_interfaces::msg::Reference, configuration),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Vector3>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(psbmpc_interfaces::msg::Reference, velocity),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Reference_message_members = {
  "psbmpc_interfaces::msg",  // message namespace
  "Reference",  // message name
  2,  // number of fields
  sizeof(psbmpc_interfaces::msg::Reference),
  Reference_message_member_array,  // message members
  Reference_init_function,  // function to initialize message memory (memory has to be allocated)
  Reference_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Reference_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Reference_message_members,
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
get_message_type_support_handle<psbmpc_interfaces::msg::Reference>()
{
  return &::psbmpc_interfaces::msg::rosidl_typesupport_introspection_cpp::Reference_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, psbmpc_interfaces, msg, Reference)() {
  return &::psbmpc_interfaces::msg::rosidl_typesupport_introspection_cpp::Reference_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
