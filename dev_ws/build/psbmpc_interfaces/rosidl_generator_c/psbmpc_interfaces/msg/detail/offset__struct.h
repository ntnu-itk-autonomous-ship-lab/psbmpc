// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from psbmpc_interfaces:msg/Offset.idl
// generated code does not contain a copyright notice

#ifndef PSBMPC_INTERFACES__MSG__DETAIL__OFFSET__STRUCT_H_
#define PSBMPC_INTERFACES__MSG__DETAIL__OFFSET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/Offset in the package psbmpc_interfaces.
typedef struct psbmpc_interfaces__msg__Offset
{
  double u_m;
  double chi_m;
} psbmpc_interfaces__msg__Offset;

// Struct for a sequence of psbmpc_interfaces__msg__Offset.
typedef struct psbmpc_interfaces__msg__Offset__Sequence
{
  psbmpc_interfaces__msg__Offset * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} psbmpc_interfaces__msg__Offset__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PSBMPC_INTERFACES__MSG__DETAIL__OFFSET__STRUCT_H_
