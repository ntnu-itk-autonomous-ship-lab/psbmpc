// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from psbmpc_interfaces:msg/Trajectory6D.idl
// generated code does not contain a copyright notice

#ifndef PSBMPC_INTERFACES__MSG__DETAIL__TRAJECTORY6_D__TRAITS_HPP_
#define PSBMPC_INTERFACES__MSG__DETAIL__TRAJECTORY6_D__TRAITS_HPP_

#include "psbmpc_interfaces/msg/detail/trajectory6_d__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<psbmpc_interfaces::msg::Trajectory6D>()
{
  return "psbmpc_interfaces::msg::Trajectory6D";
}

template<>
inline const char * name<psbmpc_interfaces::msg::Trajectory6D>()
{
  return "psbmpc_interfaces/msg/Trajectory6D";
}

template<>
struct has_fixed_size<psbmpc_interfaces::msg::Trajectory6D>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<psbmpc_interfaces::msg::Trajectory6D>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<psbmpc_interfaces::msg::Trajectory6D>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PSBMPC_INTERFACES__MSG__DETAIL__TRAJECTORY6_D__TRAITS_HPP_
