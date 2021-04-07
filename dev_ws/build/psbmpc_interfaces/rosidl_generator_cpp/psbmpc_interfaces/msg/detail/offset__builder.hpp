// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from psbmpc_interfaces:msg/Offset.idl
// generated code does not contain a copyright notice

#ifndef PSBMPC_INTERFACES__MSG__DETAIL__OFFSET__BUILDER_HPP_
#define PSBMPC_INTERFACES__MSG__DETAIL__OFFSET__BUILDER_HPP_

#include "psbmpc_interfaces/msg/detail/offset__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace psbmpc_interfaces
{

namespace msg
{

namespace builder
{

class Init_Offset_chi_m
{
public:
  explicit Init_Offset_chi_m(::psbmpc_interfaces::msg::Offset & msg)
  : msg_(msg)
  {}
  ::psbmpc_interfaces::msg::Offset chi_m(::psbmpc_interfaces::msg::Offset::_chi_m_type arg)
  {
    msg_.chi_m = std::move(arg);
    return std::move(msg_);
  }

private:
  ::psbmpc_interfaces::msg::Offset msg_;
};

class Init_Offset_u_m
{
public:
  Init_Offset_u_m()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Offset_chi_m u_m(::psbmpc_interfaces::msg::Offset::_u_m_type arg)
  {
    msg_.u_m = std::move(arg);
    return Init_Offset_chi_m(msg_);
  }

private:
  ::psbmpc_interfaces::msg::Offset msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::psbmpc_interfaces::msg::Offset>()
{
  return psbmpc_interfaces::msg::builder::Init_Offset_u_m();
}

}  // namespace psbmpc_interfaces

#endif  // PSBMPC_INTERFACES__MSG__DETAIL__OFFSET__BUILDER_HPP_
