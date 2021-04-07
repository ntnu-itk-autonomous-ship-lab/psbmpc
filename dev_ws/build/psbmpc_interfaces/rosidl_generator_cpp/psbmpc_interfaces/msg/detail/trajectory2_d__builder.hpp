// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from psbmpc_interfaces:msg/Trajectory2D.idl
// generated code does not contain a copyright notice

#ifndef PSBMPC_INTERFACES__MSG__DETAIL__TRAJECTORY2_D__BUILDER_HPP_
#define PSBMPC_INTERFACES__MSG__DETAIL__TRAJECTORY2_D__BUILDER_HPP_

#include "psbmpc_interfaces/msg/detail/trajectory2_d__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace psbmpc_interfaces
{

namespace msg
{

namespace builder
{

class Init_Trajectory2D_waypoints
{
public:
  explicit Init_Trajectory2D_waypoints(::psbmpc_interfaces::msg::Trajectory2D & msg)
  : msg_(msg)
  {}
  ::psbmpc_interfaces::msg::Trajectory2D waypoints(::psbmpc_interfaces::msg::Trajectory2D::_waypoints_type arg)
  {
    msg_.waypoints = std::move(arg);
    return std::move(msg_);
  }

private:
  ::psbmpc_interfaces::msg::Trajectory2D msg_;
};

class Init_Trajectory2D_header
{
public:
  Init_Trajectory2D_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Trajectory2D_waypoints header(::psbmpc_interfaces::msg::Trajectory2D::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Trajectory2D_waypoints(msg_);
  }

private:
  ::psbmpc_interfaces::msg::Trajectory2D msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::psbmpc_interfaces::msg::Trajectory2D>()
{
  return psbmpc_interfaces::msg::builder::Init_Trajectory2D_header();
}

}  // namespace psbmpc_interfaces

#endif  // PSBMPC_INTERFACES__MSG__DETAIL__TRAJECTORY2_D__BUILDER_HPP_
