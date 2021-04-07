// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from psbmpc_interfaces:msg/KinematicEstimateStamped.idl
// generated code does not contain a copyright notice

#ifndef PSBMPC_INTERFACES__MSG__DETAIL__KINEMATIC_ESTIMATE_STAMPED__BUILDER_HPP_
#define PSBMPC_INTERFACES__MSG__DETAIL__KINEMATIC_ESTIMATE_STAMPED__BUILDER_HPP_

#include "psbmpc_interfaces/msg/detail/kinematic_estimate_stamped__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace psbmpc_interfaces
{

namespace msg
{

namespace builder
{

class Init_KinematicEstimateStamped_kinematic_estimate
{
public:
  explicit Init_KinematicEstimateStamped_kinematic_estimate(::psbmpc_interfaces::msg::KinematicEstimateStamped & msg)
  : msg_(msg)
  {}
  ::psbmpc_interfaces::msg::KinematicEstimateStamped kinematic_estimate(::psbmpc_interfaces::msg::KinematicEstimateStamped::_kinematic_estimate_type arg)
  {
    msg_.kinematic_estimate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::psbmpc_interfaces::msg::KinematicEstimateStamped msg_;
};

class Init_KinematicEstimateStamped_header
{
public:
  Init_KinematicEstimateStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_KinematicEstimateStamped_kinematic_estimate header(::psbmpc_interfaces::msg::KinematicEstimateStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_KinematicEstimateStamped_kinematic_estimate(msg_);
  }

private:
  ::psbmpc_interfaces::msg::KinematicEstimateStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::psbmpc_interfaces::msg::KinematicEstimateStamped>()
{
  return psbmpc_interfaces::msg::builder::Init_KinematicEstimateStamped_header();
}

}  // namespace psbmpc_interfaces

#endif  // PSBMPC_INTERFACES__MSG__DETAIL__KINEMATIC_ESTIMATE_STAMPED__BUILDER_HPP_
