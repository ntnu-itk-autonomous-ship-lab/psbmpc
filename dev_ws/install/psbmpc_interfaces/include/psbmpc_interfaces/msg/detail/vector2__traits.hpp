// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from psbmpc_interfaces:msg/Vector2.idl
// generated code does not contain a copyright notice

#ifndef PSBMPC_INTERFACES__MSG__DETAIL__VECTOR2__TRAITS_HPP_
#define PSBMPC_INTERFACES__MSG__DETAIL__VECTOR2__TRAITS_HPP_

#include "psbmpc_interfaces/msg/detail/vector2__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<psbmpc_interfaces::msg::Vector2>()
{
  return "psbmpc_interfaces::msg::Vector2";
}

template<>
inline const char * name<psbmpc_interfaces::msg::Vector2>()
{
  return "psbmpc_interfaces/msg/Vector2";
}

template<>
struct has_fixed_size<psbmpc_interfaces::msg::Vector2>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<psbmpc_interfaces::msg::Vector2>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<psbmpc_interfaces::msg::Vector2>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PSBMPC_INTERFACES__MSG__DETAIL__VECTOR2__TRAITS_HPP_
