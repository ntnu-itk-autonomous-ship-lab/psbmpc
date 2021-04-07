// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from psbmpc_interfaces:msg/KinematicEstimateStamped.idl
// generated code does not contain a copyright notice

#ifndef PSBMPC_INTERFACES__MSG__DETAIL__KINEMATIC_ESTIMATE_STAMPED__TRAITS_HPP_
#define PSBMPC_INTERFACES__MSG__DETAIL__KINEMATIC_ESTIMATE_STAMPED__TRAITS_HPP_

#include "psbmpc_interfaces/msg/detail/kinematic_estimate_stamped__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'kinematic_estimate'
#include "psbmpc_interfaces/msg/detail/kinematic_estimate__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<psbmpc_interfaces::msg::KinematicEstimateStamped>()
{
  return "psbmpc_interfaces::msg::KinematicEstimateStamped";
}

template<>
inline const char * name<psbmpc_interfaces::msg::KinematicEstimateStamped>()
{
  return "psbmpc_interfaces/msg/KinematicEstimateStamped";
}

template<>
struct has_fixed_size<psbmpc_interfaces::msg::KinematicEstimateStamped>
  : std::integral_constant<bool, has_fixed_size<psbmpc_interfaces::msg::KinematicEstimate>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<psbmpc_interfaces::msg::KinematicEstimateStamped>
  : std::integral_constant<bool, has_bounded_size<psbmpc_interfaces::msg::KinematicEstimate>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<psbmpc_interfaces::msg::KinematicEstimateStamped>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PSBMPC_INTERFACES__MSG__DETAIL__KINEMATIC_ESTIMATE_STAMPED__TRAITS_HPP_
