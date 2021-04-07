// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from psbmpc_interfaces:msg/Reference.idl
// generated code does not contain a copyright notice

#ifndef PSBMPC_INTERFACES__MSG__DETAIL__REFERENCE__TRAITS_HPP_
#define PSBMPC_INTERFACES__MSG__DETAIL__REFERENCE__TRAITS_HPP_

#include "psbmpc_interfaces/msg/detail/reference__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'configuration'
// Member 'velocity'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<psbmpc_interfaces::msg::Reference>()
{
  return "psbmpc_interfaces::msg::Reference";
}

template<>
inline const char * name<psbmpc_interfaces::msg::Reference>()
{
  return "psbmpc_interfaces/msg/Reference";
}

template<>
struct has_fixed_size<psbmpc_interfaces::msg::Reference>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct has_bounded_size<psbmpc_interfaces::msg::Reference>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct is_message<psbmpc_interfaces::msg::Reference>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PSBMPC_INTERFACES__MSG__DETAIL__REFERENCE__TRAITS_HPP_
