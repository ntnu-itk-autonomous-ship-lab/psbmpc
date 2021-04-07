// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from psbmpc_interfaces:msg/KinematicEstimateStamped.idl
// generated code does not contain a copyright notice

#ifndef PSBMPC_INTERFACES__MSG__DETAIL__KINEMATIC_ESTIMATE_STAMPED__STRUCT_H_
#define PSBMPC_INTERFACES__MSG__DETAIL__KINEMATIC_ESTIMATE_STAMPED__STRUCT_H_

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
// Member 'kinematic_estimate'
#include "psbmpc_interfaces/msg/detail/kinematic_estimate__struct.h"

// Struct defined in msg/KinematicEstimateStamped in the package psbmpc_interfaces.
typedef struct psbmpc_interfaces__msg__KinematicEstimateStamped
{
  std_msgs__msg__Header header;
  psbmpc_interfaces__msg__KinematicEstimate kinematic_estimate;
} psbmpc_interfaces__msg__KinematicEstimateStamped;

// Struct for a sequence of psbmpc_interfaces__msg__KinematicEstimateStamped.
typedef struct psbmpc_interfaces__msg__KinematicEstimateStamped__Sequence
{
  psbmpc_interfaces__msg__KinematicEstimateStamped * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} psbmpc_interfaces__msg__KinematicEstimateStamped__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PSBMPC_INTERFACES__MSG__DETAIL__KINEMATIC_ESTIMATE_STAMPED__STRUCT_H_
