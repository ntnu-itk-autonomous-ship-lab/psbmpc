// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from psbmpc_interfaces:msg/KinematicEstimateStamped.idl
// generated code does not contain a copyright notice
#include "psbmpc_interfaces/msg/detail/kinematic_estimate_stamped__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `kinematic_estimate`
#include "psbmpc_interfaces/msg/detail/kinematic_estimate__functions.h"

bool
psbmpc_interfaces__msg__KinematicEstimateStamped__init(psbmpc_interfaces__msg__KinematicEstimateStamped * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    psbmpc_interfaces__msg__KinematicEstimateStamped__fini(msg);
    return false;
  }
  // kinematic_estimate
  if (!psbmpc_interfaces__msg__KinematicEstimate__init(&msg->kinematic_estimate)) {
    psbmpc_interfaces__msg__KinematicEstimateStamped__fini(msg);
    return false;
  }
  return true;
}

void
psbmpc_interfaces__msg__KinematicEstimateStamped__fini(psbmpc_interfaces__msg__KinematicEstimateStamped * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // kinematic_estimate
  psbmpc_interfaces__msg__KinematicEstimate__fini(&msg->kinematic_estimate);
}

psbmpc_interfaces__msg__KinematicEstimateStamped *
psbmpc_interfaces__msg__KinematicEstimateStamped__create()
{
  psbmpc_interfaces__msg__KinematicEstimateStamped * msg = (psbmpc_interfaces__msg__KinematicEstimateStamped *)malloc(sizeof(psbmpc_interfaces__msg__KinematicEstimateStamped));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(psbmpc_interfaces__msg__KinematicEstimateStamped));
  bool success = psbmpc_interfaces__msg__KinematicEstimateStamped__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
psbmpc_interfaces__msg__KinematicEstimateStamped__destroy(psbmpc_interfaces__msg__KinematicEstimateStamped * msg)
{
  if (msg) {
    psbmpc_interfaces__msg__KinematicEstimateStamped__fini(msg);
  }
  free(msg);
}


bool
psbmpc_interfaces__msg__KinematicEstimateStamped__Sequence__init(psbmpc_interfaces__msg__KinematicEstimateStamped__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  psbmpc_interfaces__msg__KinematicEstimateStamped * data = NULL;
  if (size) {
    data = (psbmpc_interfaces__msg__KinematicEstimateStamped *)calloc(size, sizeof(psbmpc_interfaces__msg__KinematicEstimateStamped));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = psbmpc_interfaces__msg__KinematicEstimateStamped__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        psbmpc_interfaces__msg__KinematicEstimateStamped__fini(&data[i - 1]);
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
psbmpc_interfaces__msg__KinematicEstimateStamped__Sequence__fini(psbmpc_interfaces__msg__KinematicEstimateStamped__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      psbmpc_interfaces__msg__KinematicEstimateStamped__fini(&array->data[i]);
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

psbmpc_interfaces__msg__KinematicEstimateStamped__Sequence *
psbmpc_interfaces__msg__KinematicEstimateStamped__Sequence__create(size_t size)
{
  psbmpc_interfaces__msg__KinematicEstimateStamped__Sequence * array = (psbmpc_interfaces__msg__KinematicEstimateStamped__Sequence *)malloc(sizeof(psbmpc_interfaces__msg__KinematicEstimateStamped__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = psbmpc_interfaces__msg__KinematicEstimateStamped__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
psbmpc_interfaces__msg__KinematicEstimateStamped__Sequence__destroy(psbmpc_interfaces__msg__KinematicEstimateStamped__Sequence * array)
{
  if (array) {
    psbmpc_interfaces__msg__KinematicEstimateStamped__Sequence__fini(array);
  }
  free(array);
}
