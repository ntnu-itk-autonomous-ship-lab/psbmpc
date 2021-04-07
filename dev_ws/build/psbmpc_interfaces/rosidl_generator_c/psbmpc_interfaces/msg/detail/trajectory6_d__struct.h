// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from psbmpc_interfaces:msg/Trajectory6D.idl
// generated code does not contain a copyright notice

#ifndef PSBMPC_INTERFACES__MSG__DETAIL__TRAJECTORY6_D__STRUCT_H_
#define PSBMPC_INTERFACES__MSG__DETAIL__TRAJECTORY6_D__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'predicted_trajectory'
#include "psbmpc_interfaces/msg/detail/reference__struct.h"

// Struct defined in msg/Trajectory6D in the package psbmpc_interfaces.
typedef struct psbmpc_interfaces__msg__Trajectory6D
{
  std_msgs__msg__Header header;
  psbmpc_interfaces__msg__Reference__Sequence predicted_trajectory;
} psbmpc_interfaces__msg__Trajectory6D;

// Struct for a sequence of psbmpc_interfaces__msg__Trajectory6D.
typedef struct psbmpc_interfaces__msg__Trajectory6D__Sequence
{
  psbmpc_interfaces__msg__Trajectory6D * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} psbmpc_interfaces__msg__Trajectory6D__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PSBMPC_INTERFACES__MSG__DETAIL__TRAJECTORY6_D__STRUCT_H_
