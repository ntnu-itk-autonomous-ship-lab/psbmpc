// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from psbmpc_interfaces:msg/Covariance2.idl
// generated code does not contain a copyright notice

#ifndef PSBMPC_INTERFACES__MSG__DETAIL__COVARIANCE2__BUILDER_HPP_
#define PSBMPC_INTERFACES__MSG__DETAIL__COVARIANCE2__BUILDER_HPP_

#include "psbmpc_interfaces/msg/detail/covariance2__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace psbmpc_interfaces
{

namespace msg
{

namespace builder
{

class Init_Covariance2_cor_xy
{
public:
  explicit Init_Covariance2_cor_xy(::psbmpc_interfaces::msg::Covariance2 & msg)
  : msg_(msg)
  {}
  ::psbmpc_interfaces::msg::Covariance2 cor_xy(::psbmpc_interfaces::msg::Covariance2::_cor_xy_type arg)
  {
    msg_.cor_xy = std::move(arg);
    return std::move(msg_);
  }

private:
  ::psbmpc_interfaces::msg::Covariance2 msg_;
};

class Init_Covariance2_var_y
{
public:
  explicit Init_Covariance2_var_y(::psbmpc_interfaces::msg::Covariance2 & msg)
  : msg_(msg)
  {}
  Init_Covariance2_cor_xy var_y(::psbmpc_interfaces::msg::Covariance2::_var_y_type arg)
  {
    msg_.var_y = std::move(arg);
    return Init_Covariance2_cor_xy(msg_);
  }

private:
  ::psbmpc_interfaces::msg::Covariance2 msg_;
};

class Init_Covariance2_var_x
{
public:
  Init_Covariance2_var_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Covariance2_var_y var_x(::psbmpc_interfaces::msg::Covariance2::_var_x_type arg)
  {
    msg_.var_x = std::move(arg);
    return Init_Covariance2_var_y(msg_);
  }

private:
  ::psbmpc_interfaces::msg::Covariance2 msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::psbmpc_interfaces::msg::Covariance2>()
{
  return psbmpc_interfaces::msg::builder::Init_Covariance2_var_x();
}

}  // namespace psbmpc_interfaces

#endif  // PSBMPC_INTERFACES__MSG__DETAIL__COVARIANCE2__BUILDER_HPP_
