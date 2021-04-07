// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from psbmpc_interfaces:msg/KinematicEstimate.idl
// generated code does not contain a copyright notice

#ifndef PSBMPC_INTERFACES__MSG__DETAIL__KINEMATIC_ESTIMATE__TRAITS_HPP_
#define PSBMPC_INTERFACES__MSG__DETAIL__KINEMATIC_ESTIMATE__TRAITS_HPP_

#include "psbmpc_interfaces/msg/detail/kinematic_estimate__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'pos_est'
// Member 'vel_est'
#include "psbmpc_interfaces/msg/detail/vector2__traits.hpp"
// Member 'pos_cov'
// Member 'vel_cov'
#include "psbmpc_interfaces/msg/detail/covariance2__traits.hpp"
// Member 'pos_vel_corr'
#include "psbmpc_interfaces/msg/detail/cross_covariance2__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<psbmpc_interfaces::msg::KinematicEstimate>()
{
  return "psbmpc_interfaces::msg::KinematicEstimate";
}

template<>
inline const char * name<psbmpc_interfaces::msg::KinematicEstimate>()
{
  return "psbmpc_interfaces/msg/KinematicEstimate";
}

template<>
struct has_fixed_size<psbmpc_interfaces::msg::KinematicEstimate>
  : std::integral_constant<bool, has_fixed_size<psbmpc_interfaces::msg::Covariance2>::value && has_fixed_size<psbmpc_interfaces::msg::CrossCovariance2>::value && has_fixed_size<psbmpc_interfaces::msg::Vector2>::value> {};

template<>
struct has_bounded_size<psbmpc_interfaces::msg::KinematicEstimate>
  : std::integral_constant<bool, has_bounded_size<psbmpc_interfaces::msg::Covariance2>::value && has_bounded_size<psbmpc_interfaces::msg::CrossCovariance2>::value && has_bounded_size<psbmpc_interfaces::msg::Vector2>::value> {};

template<>
struct is_message<psbmpc_interfaces::msg::KinematicEstimate>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PSBMPC_INTERFACES__MSG__DETAIL__KINEMATIC_ESTIMATE__TRAITS_HPP_
