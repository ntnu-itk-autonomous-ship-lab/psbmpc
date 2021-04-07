// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from psbmpc_interfaces:msg/KinematicEstimate.idl
// generated code does not contain a copyright notice

#ifndef PSBMPC_INTERFACES__MSG__DETAIL__KINEMATIC_ESTIMATE__BUILDER_HPP_
#define PSBMPC_INTERFACES__MSG__DETAIL__KINEMATIC_ESTIMATE__BUILDER_HPP_

#include "psbmpc_interfaces/msg/detail/kinematic_estimate__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace psbmpc_interfaces
{

namespace msg
{

namespace builder
{

class Init_KinematicEstimate_pos_vel_corr
{
public:
  explicit Init_KinematicEstimate_pos_vel_corr(::psbmpc_interfaces::msg::KinematicEstimate & msg)
  : msg_(msg)
  {}
  ::psbmpc_interfaces::msg::KinematicEstimate pos_vel_corr(::psbmpc_interfaces::msg::KinematicEstimate::_pos_vel_corr_type arg)
  {
    msg_.pos_vel_corr = std::move(arg);
    return std::move(msg_);
  }

private:
  ::psbmpc_interfaces::msg::KinematicEstimate msg_;
};

class Init_KinematicEstimate_vel_cov
{
public:
  explicit Init_KinematicEstimate_vel_cov(::psbmpc_interfaces::msg::KinematicEstimate & msg)
  : msg_(msg)
  {}
  Init_KinematicEstimate_pos_vel_corr vel_cov(::psbmpc_interfaces::msg::KinematicEstimate::_vel_cov_type arg)
  {
    msg_.vel_cov = std::move(arg);
    return Init_KinematicEstimate_pos_vel_corr(msg_);
  }

private:
  ::psbmpc_interfaces::msg::KinematicEstimate msg_;
};

class Init_KinematicEstimate_pos_cov
{
public:
  explicit Init_KinematicEstimate_pos_cov(::psbmpc_interfaces::msg::KinematicEstimate & msg)
  : msg_(msg)
  {}
  Init_KinematicEstimate_vel_cov pos_cov(::psbmpc_interfaces::msg::KinematicEstimate::_pos_cov_type arg)
  {
    msg_.pos_cov = std::move(arg);
    return Init_KinematicEstimate_vel_cov(msg_);
  }

private:
  ::psbmpc_interfaces::msg::KinematicEstimate msg_;
};

class Init_KinematicEstimate_vel_est
{
public:
  explicit Init_KinematicEstimate_vel_est(::psbmpc_interfaces::msg::KinematicEstimate & msg)
  : msg_(msg)
  {}
  Init_KinematicEstimate_pos_cov vel_est(::psbmpc_interfaces::msg::KinematicEstimate::_vel_est_type arg)
  {
    msg_.vel_est = std::move(arg);
    return Init_KinematicEstimate_pos_cov(msg_);
  }

private:
  ::psbmpc_interfaces::msg::KinematicEstimate msg_;
};

class Init_KinematicEstimate_pos_est
{
public:
  Init_KinematicEstimate_pos_est()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_KinematicEstimate_vel_est pos_est(::psbmpc_interfaces::msg::KinematicEstimate::_pos_est_type arg)
  {
    msg_.pos_est = std::move(arg);
    return Init_KinematicEstimate_vel_est(msg_);
  }

private:
  ::psbmpc_interfaces::msg::KinematicEstimate msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::psbmpc_interfaces::msg::KinematicEstimate>()
{
  return psbmpc_interfaces::msg::builder::Init_KinematicEstimate_pos_est();
}

}  // namespace psbmpc_interfaces

#endif  // PSBMPC_INTERFACES__MSG__DETAIL__KINEMATIC_ESTIMATE__BUILDER_HPP_
