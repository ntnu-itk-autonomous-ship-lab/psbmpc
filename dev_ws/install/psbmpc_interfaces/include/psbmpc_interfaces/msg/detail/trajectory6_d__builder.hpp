// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from psbmpc_interfaces:msg/Trajectory6D.idl
// generated code does not contain a copyright notice

#ifndef PSBMPC_INTERFACES__MSG__DETAIL__TRAJECTORY6_D__BUILDER_HPP_
#define PSBMPC_INTERFACES__MSG__DETAIL__TRAJECTORY6_D__BUILDER_HPP_

#include "psbmpc_interfaces/msg/detail/trajectory6_d__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace psbmpc_interfaces
{

namespace msg
{

namespace builder
{

class Init_Trajectory6D_predicted_trajectory
{
public:
  explicit Init_Trajectory6D_predicted_trajectory(::psbmpc_interfaces::msg::Trajectory6D & msg)
  : msg_(msg)
  {}
  ::psbmpc_interfaces::msg::Trajectory6D predicted_trajectory(::psbmpc_interfaces::msg::Trajectory6D::_predicted_trajectory_type arg)
  {
    msg_.predicted_trajectory = std::move(arg);
    return std::move(msg_);
  }

private:
  ::psbmpc_interfaces::msg::Trajectory6D msg_;
};

class Init_Trajectory6D_header
{
public:
  Init_Trajectory6D_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Trajectory6D_predicted_trajectory header(::psbmpc_interfaces::msg::Trajectory6D::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Trajectory6D_predicted_trajectory(msg_);
  }

private:
  ::psbmpc_interfaces::msg::Trajectory6D msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::psbmpc_interfaces::msg::Trajectory6D>()
{
  return psbmpc_interfaces::msg::builder::Init_Trajectory6D_header();
}

}  // namespace psbmpc_interfaces

#endif  // PSBMPC_INTERFACES__MSG__DETAIL__TRAJECTORY6_D__BUILDER_HPP_
