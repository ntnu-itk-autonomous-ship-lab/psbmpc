// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from psbmpc_interfaces:msg/Vector2.idl
// generated code does not contain a copyright notice

#ifndef PSBMPC_INTERFACES__MSG__DETAIL__VECTOR2__BUILDER_HPP_
#define PSBMPC_INTERFACES__MSG__DETAIL__VECTOR2__BUILDER_HPP_

#include "psbmpc_interfaces/msg/detail/vector2__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace psbmpc_interfaces
{

namespace msg
{

namespace builder
{

class Init_Vector2_y
{
public:
  explicit Init_Vector2_y(::psbmpc_interfaces::msg::Vector2 & msg)
  : msg_(msg)
  {}
  ::psbmpc_interfaces::msg::Vector2 y(::psbmpc_interfaces::msg::Vector2::_y_type arg)
  {
    msg_.y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::psbmpc_interfaces::msg::Vector2 msg_;
};

class Init_Vector2_x
{
public:
  Init_Vector2_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Vector2_y x(::psbmpc_interfaces::msg::Vector2::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Vector2_y(msg_);
  }

private:
  ::psbmpc_interfaces::msg::Vector2 msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::psbmpc_interfaces::msg::Vector2>()
{
  return psbmpc_interfaces::msg::builder::Init_Vector2_x();
}

}  // namespace psbmpc_interfaces

#endif  // PSBMPC_INTERFACES__MSG__DETAIL__VECTOR2__BUILDER_HPP_
