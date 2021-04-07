// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from psbmpc_interfaces:msg/Vector2.idl
// generated code does not contain a copyright notice

#ifndef PSBMPC_INTERFACES__MSG__DETAIL__VECTOR2__STRUCT_H_
#define PSBMPC_INTERFACES__MSG__DETAIL__VECTOR2__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/Vector2 in the package psbmpc_interfaces.
typedef struct psbmpc_interfaces__msg__Vector2
{
  double x;
  double y;
} psbmpc_interfaces__msg__Vector2;

// Struct for a sequence of psbmpc_interfaces__msg__Vector2.
typedef struct psbmpc_interfaces__msg__Vector2__Sequence
{
  psbmpc_interfaces__msg__Vector2 * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} psbmpc_interfaces__msg__Vector2__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PSBMPC_INTERFACES__MSG__DETAIL__VECTOR2__STRUCT_H_
