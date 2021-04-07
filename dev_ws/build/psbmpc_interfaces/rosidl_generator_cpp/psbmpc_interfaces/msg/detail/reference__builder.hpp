// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from psbmpc_interfaces:msg/Reference.idl
// generated code does not contain a copyright notice

#ifndef PSBMPC_INTERFACES__MSG__DETAIL__REFERENCE__BUILDER_HPP_
#define PSBMPC_INTERFACES__MSG__DETAIL__REFERENCE__BUILDER_HPP_

#include "psbmpc_interfaces/msg/detail/reference__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace psbmpc_interfaces
{

namespace msg
{

namespace builder
{

class Init_Reference_velocity
{
public:
  explicit Init_Reference_velocity(::psbmpc_interfaces::msg::Reference & msg)
  : msg_(msg)
  {}
  ::psbmpc_interfaces::msg::Reference velocity(::psbmpc_interfaces::msg::Reference::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::psbmpc_interfaces::msg::Reference msg_;
};

class Init_Reference_configuration
{
public:
  Init_Reference_configuration()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Reference_velocity configuration(::psbmpc_interfaces::msg::Reference::_configuration_type arg)
  {
    msg_.configuration = std::move(arg);
    return Init_Reference_velocity(msg_);
  }

private:
  ::psbmpc_interfaces::msg::Reference msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::psbmpc_interfaces::msg::Reference>()
{
  return psbmpc_interfaces::msg::builder::Init_Reference_configuration();
}

}  // namespace psbmpc_interfaces

#endif  // PSBMPC_INTERFACES__MSG__DETAIL__REFERENCE__BUILDER_HPP_
