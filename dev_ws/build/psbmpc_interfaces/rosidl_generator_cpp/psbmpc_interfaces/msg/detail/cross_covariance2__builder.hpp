// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from psbmpc_interfaces:msg/CrossCovariance2.idl
// generated code does not contain a copyright notice

#ifndef PSBMPC_INTERFACES__MSG__DETAIL__CROSS_COVARIANCE2__BUILDER_HPP_
#define PSBMPC_INTERFACES__MSG__DETAIL__CROSS_COVARIANCE2__BUILDER_HPP_

#include "psbmpc_interfaces/msg/detail/cross_covariance2__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace psbmpc_interfaces
{

namespace msg
{

namespace builder
{

class Init_CrossCovariance2_cor_py_vy
{
public:
  explicit Init_CrossCovariance2_cor_py_vy(::psbmpc_interfaces::msg::CrossCovariance2 & msg)
  : msg_(msg)
  {}
  ::psbmpc_interfaces::msg::CrossCovariance2 cor_py_vy(::psbmpc_interfaces::msg::CrossCovariance2::_cor_py_vy_type arg)
  {
    msg_.cor_py_vy = std::move(arg);
    return std::move(msg_);
  }

private:
  ::psbmpc_interfaces::msg::CrossCovariance2 msg_;
};

class Init_CrossCovariance2_cor_py_vx
{
public:
  explicit Init_CrossCovariance2_cor_py_vx(::psbmpc_interfaces::msg::CrossCovariance2 & msg)
  : msg_(msg)
  {}
  Init_CrossCovariance2_cor_py_vy cor_py_vx(::psbmpc_interfaces::msg::CrossCovariance2::_cor_py_vx_type arg)
  {
    msg_.cor_py_vx = std::move(arg);
    return Init_CrossCovariance2_cor_py_vy(msg_);
  }

private:
  ::psbmpc_interfaces::msg::CrossCovariance2 msg_;
};

class Init_CrossCovariance2_cor_px_vy
{
public:
  explicit Init_CrossCovariance2_cor_px_vy(::psbmpc_interfaces::msg::CrossCovariance2 & msg)
  : msg_(msg)
  {}
  Init_CrossCovariance2_cor_py_vx cor_px_vy(::psbmpc_interfaces::msg::CrossCovariance2::_cor_px_vy_type arg)
  {
    msg_.cor_px_vy = std::move(arg);
    return Init_CrossCovariance2_cor_py_vx(msg_);
  }

private:
  ::psbmpc_interfaces::msg::CrossCovariance2 msg_;
};

class Init_CrossCovariance2_cor_px_vx
{
public:
  Init_CrossCovariance2_cor_px_vx()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CrossCovariance2_cor_px_vy cor_px_vx(::psbmpc_interfaces::msg::CrossCovariance2::_cor_px_vx_type arg)
  {
    msg_.cor_px_vx = std::move(arg);
    return Init_CrossCovariance2_cor_px_vy(msg_);
  }

private:
  ::psbmpc_interfaces::msg::CrossCovariance2 msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::psbmpc_interfaces::msg::CrossCovariance2>()
{
  return psbmpc_interfaces::msg::builder::Init_CrossCovariance2_cor_px_vx();
}

}  // namespace psbmpc_interfaces

#endif  // PSBMPC_INTERFACES__MSG__DETAIL__CROSS_COVARIANCE2__BUILDER_HPP_
