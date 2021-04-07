// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from psbmpc_interfaces:msg/Reference.idl
// generated code does not contain a copyright notice

#ifndef PSBMPC_INTERFACES__MSG__DETAIL__REFERENCE__STRUCT_H_
#define PSBMPC_INTERFACES__MSG__DETAIL__REFERENCE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'configuration'
// Member 'velocity'
#include "geometry_msgs/msg/detail/vector3__struct.h"

// Struct defined in msg/Reference in the package psbmpc_interfaces.
typedef struct psbmpc_interfaces__msg__Reference
{
  geometry_msgs__msg__Vector3 configuration;
  geometry_msgs__msg__Vector3 velocity;
} psbmpc_interfaces__msg__Reference;

// Struct for a sequence of psbmpc_interfaces__msg__Reference.
typedef struct psbmpc_interfaces__msg__Reference__Sequence
{
  psbmpc_interfaces__msg__Reference * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} psbmpc_interfaces__msg__Reference__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PSBMPC_INTERFACES__MSG__DETAIL__REFERENCE__STRUCT_H_
