// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from psbmpc_interfaces:msg/KinematicEstimate.idl
// generated code does not contain a copyright notice

#ifndef PSBMPC_INTERFACES__MSG__DETAIL__KINEMATIC_ESTIMATE__STRUCT_H_
#define PSBMPC_INTERFACES__MSG__DETAIL__KINEMATIC_ESTIMATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pos_est'
// Member 'vel_est'
#include "psbmpc_interfaces/msg/detail/vector2__struct.h"
// Member 'pos_cov'
// Member 'vel_cov'
#include "psbmpc_interfaces/msg/detail/covariance2__struct.h"
// Member 'pos_vel_corr'
#include "psbmpc_interfaces/msg/detail/cross_covariance2__struct.h"

// Struct defined in msg/KinematicEstimate in the package psbmpc_interfaces.
typedef struct psbmpc_interfaces__msg__KinematicEstimate
{
  psbmpc_interfaces__msg__Vector2 pos_est;
  psbmpc_interfaces__msg__Vector2 vel_est;
  psbmpc_interfaces__msg__Covariance2 pos_cov;
  psbmpc_interfaces__msg__Covariance2 vel_cov;
  psbmpc_interfaces__msg__CrossCovariance2 pos_vel_corr;
} psbmpc_interfaces__msg__KinematicEstimate;

// Struct for a sequence of psbmpc_interfaces__msg__KinematicEstimate.
typedef struct psbmpc_interfaces__msg__KinematicEstimate__Sequence
{
  psbmpc_interfaces__msg__KinematicEstimate * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} psbmpc_interfaces__msg__KinematicEstimate__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PSBMPC_INTERFACES__MSG__DETAIL__KINEMATIC_ESTIMATE__STRUCT_H_
