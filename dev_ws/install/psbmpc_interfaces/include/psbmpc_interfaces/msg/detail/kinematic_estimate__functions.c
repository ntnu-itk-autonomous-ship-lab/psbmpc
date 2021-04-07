// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from psbmpc_interfaces:msg/KinematicEstimate.idl
// generated code does not contain a copyright notice
#include "psbmpc_interfaces/msg/detail/kinematic_estimate__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `pos_est`
// Member `vel_est`
#include "psbmpc_interfaces/msg/detail/vector2__functions.h"
// Member `pos_cov`
// Member `vel_cov`
#include "psbmpc_interfaces/msg/detail/covariance2__functions.h"
// Member `pos_vel_corr`
#include "psbmpc_interfaces/msg/detail/cross_covariance2__functions.h"

bool
psbmpc_interfaces__msg__KinematicEstimate__init(psbmpc_interfaces__msg__KinematicEstimate * msg)
{
  if (!msg) {
    return false;
  }
  // pos_est
  if (!psbmpc_interfaces__msg__Vector2__init(&msg->pos_est)) {
    psbmpc_interfaces__msg__KinematicEstimate__fini(msg);
    return false;
  }
  // vel_est
  if (!psbmpc_interfaces__msg__Vector2__init(&msg->vel_est)) {
    psbmpc_interfaces__msg__KinematicEstimate__fini(msg);
    return false;
  }
  // pos_cov
  if (!psbmpc_interfaces__msg__Covariance2__init(&msg->pos_cov)) {
    psbmpc_interfaces__msg__KinematicEstimate__fini(msg);
    return false;
  }
  // vel_cov
  if (!psbmpc_interfaces__msg__Covariance2__init(&msg->vel_cov)) {
    psbmpc_interfaces__msg__KinematicEstimate__fini(msg);
    return false;
  }
  // pos_vel_corr
  if (!psbmpc_interfaces__msg__CrossCovariance2__init(&msg->pos_vel_corr)) {
    psbmpc_interfaces__msg__KinematicEstimate__fini(msg);
    return false;
  }
  return true;
}

void
psbmpc_interfaces__msg__KinematicEstimate__fini(psbmpc_interfaces__msg__KinematicEstimate * msg)
{
  if (!msg) {
    return;
  }
  // pos_est
  psbmpc_interfaces__msg__Vector2__fini(&msg->pos_est);
  // vel_est
  psbmpc_interfaces__msg__Vector2__fini(&msg->vel_est);
  // pos_cov
  psbmpc_interfaces__msg__Covariance2__fini(&msg->pos_cov);
  // vel_cov
  psbmpc_interfaces__msg__Covariance2__fini(&msg->vel_cov);
  // pos_vel_corr
  psbmpc_interfaces__msg__CrossCovariance2__fini(&msg->pos_vel_corr);
}

psbmpc_interfaces__msg__KinematicEstimate *
psbmpc_interfaces__msg__KinematicEstimate__create()
{
  psbmpc_interfaces__msg__KinematicEstimate * msg = (psbmpc_interfaces__msg__KinematicEstimate *)malloc(sizeof(psbmpc_interfaces__msg__KinematicEstimate));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(psbmpc_interfaces__msg__KinematicEstimate));
  bool success = psbmpc_interfaces__msg__KinematicEstimate__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
psbmpc_interfaces__msg__KinematicEstimate__destroy(psbmpc_interfaces__msg__KinematicEstimate * msg)
{
  if (msg) {
    psbmpc_interfaces__msg__KinematicEstimate__fini(msg);
  }
  free(msg);
}


bool
psbmpc_interfaces__msg__KinematicEstimate__Sequence__init(psbmpc_interfaces__msg__KinematicEstimate__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  psbmpc_interfaces__msg__KinematicEstimate * data = NULL;
  if (size) {
    data = (psbmpc_interfaces__msg__KinematicEstimate *)calloc(size, sizeof(psbmpc_interfaces__msg__KinematicEstimate));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = psbmpc_interfaces__msg__KinematicEstimate__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        psbmpc_interfaces__msg__KinematicEstimate__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
psbmpc_interfaces__msg__KinematicEstimate__Sequence__fini(psbmpc_interfaces__msg__KinematicEstimate__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      psbmpc_interfaces__msg__KinematicEstimate__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

psbmpc_interfaces__msg__KinematicEstimate__Sequence *
psbmpc_interfaces__msg__KinematicEstimate__Sequence__create(size_t size)
{
  psbmpc_interfaces__msg__KinematicEstimate__Sequence * array = (psbmpc_interfaces__msg__KinematicEstimate__Sequence *)malloc(sizeof(psbmpc_interfaces__msg__KinematicEstimate__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = psbmpc_interfaces__msg__KinematicEstimate__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
psbmpc_interfaces__msg__KinematicEstimate__Sequence__destroy(psbmpc_interfaces__msg__KinematicEstimate__Sequence * array)
{
  if (array) {
    psbmpc_interfaces__msg__KinematicEstimate__Sequence__fini(array);
  }
  free(array);
}
