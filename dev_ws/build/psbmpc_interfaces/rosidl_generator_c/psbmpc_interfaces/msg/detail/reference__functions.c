// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from psbmpc_interfaces:msg/Reference.idl
// generated code does not contain a copyright notice
#include "psbmpc_interfaces/msg/detail/reference__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `configuration`
// Member `velocity`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
psbmpc_interfaces__msg__Reference__init(psbmpc_interfaces__msg__Reference * msg)
{
  if (!msg) {
    return false;
  }
  // configuration
  if (!geometry_msgs__msg__Vector3__init(&msg->configuration)) {
    psbmpc_interfaces__msg__Reference__fini(msg);
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__init(&msg->velocity)) {
    psbmpc_interfaces__msg__Reference__fini(msg);
    return false;
  }
  return true;
}

void
psbmpc_interfaces__msg__Reference__fini(psbmpc_interfaces__msg__Reference * msg)
{
  if (!msg) {
    return;
  }
  // configuration
  geometry_msgs__msg__Vector3__fini(&msg->configuration);
  // velocity
  geometry_msgs__msg__Vector3__fini(&msg->velocity);
}

psbmpc_interfaces__msg__Reference *
psbmpc_interfaces__msg__Reference__create()
{
  psbmpc_interfaces__msg__Reference * msg = (psbmpc_interfaces__msg__Reference *)malloc(sizeof(psbmpc_interfaces__msg__Reference));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(psbmpc_interfaces__msg__Reference));
  bool success = psbmpc_interfaces__msg__Reference__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
psbmpc_interfaces__msg__Reference__destroy(psbmpc_interfaces__msg__Reference * msg)
{
  if (msg) {
    psbmpc_interfaces__msg__Reference__fini(msg);
  }
  free(msg);
}


bool
psbmpc_interfaces__msg__Reference__Sequence__init(psbmpc_interfaces__msg__Reference__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  psbmpc_interfaces__msg__Reference * data = NULL;
  if (size) {
    data = (psbmpc_interfaces__msg__Reference *)calloc(size, sizeof(psbmpc_interfaces__msg__Reference));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = psbmpc_interfaces__msg__Reference__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        psbmpc_interfaces__msg__Reference__fini(&data[i - 1]);
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
psbmpc_interfaces__msg__Reference__Sequence__fini(psbmpc_interfaces__msg__Reference__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      psbmpc_interfaces__msg__Reference__fini(&array->data[i]);
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

psbmpc_interfaces__msg__Reference__Sequence *
psbmpc_interfaces__msg__Reference__Sequence__create(size_t size)
{
  psbmpc_interfaces__msg__Reference__Sequence * array = (psbmpc_interfaces__msg__Reference__Sequence *)malloc(sizeof(psbmpc_interfaces__msg__Reference__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = psbmpc_interfaces__msg__Reference__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
psbmpc_interfaces__msg__Reference__Sequence__destroy(psbmpc_interfaces__msg__Reference__Sequence * array)
{
  if (array) {
    psbmpc_interfaces__msg__Reference__Sequence__fini(array);
  }
  free(array);
}
