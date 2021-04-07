// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from psbmpc_interfaces:msg/CrossCovariance2.idl
// generated code does not contain a copyright notice
#include "psbmpc_interfaces/msg/detail/cross_covariance2__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
psbmpc_interfaces__msg__CrossCovariance2__init(psbmpc_interfaces__msg__CrossCovariance2 * msg)
{
  if (!msg) {
    return false;
  }
  // cor_px_vx
  // cor_px_vy
  // cor_py_vx
  // cor_py_vy
  return true;
}

void
psbmpc_interfaces__msg__CrossCovariance2__fini(psbmpc_interfaces__msg__CrossCovariance2 * msg)
{
  if (!msg) {
    return;
  }
  // cor_px_vx
  // cor_px_vy
  // cor_py_vx
  // cor_py_vy
}

psbmpc_interfaces__msg__CrossCovariance2 *
psbmpc_interfaces__msg__CrossCovariance2__create()
{
  psbmpc_interfaces__msg__CrossCovariance2 * msg = (psbmpc_interfaces__msg__CrossCovariance2 *)malloc(sizeof(psbmpc_interfaces__msg__CrossCovariance2));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(psbmpc_interfaces__msg__CrossCovariance2));
  bool success = psbmpc_interfaces__msg__CrossCovariance2__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
psbmpc_interfaces__msg__CrossCovariance2__destroy(psbmpc_interfaces__msg__CrossCovariance2 * msg)
{
  if (msg) {
    psbmpc_interfaces__msg__CrossCovariance2__fini(msg);
  }
  free(msg);
}


bool
psbmpc_interfaces__msg__CrossCovariance2__Sequence__init(psbmpc_interfaces__msg__CrossCovariance2__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  psbmpc_interfaces__msg__CrossCovariance2 * data = NULL;
  if (size) {
    data = (psbmpc_interfaces__msg__CrossCovariance2 *)calloc(size, sizeof(psbmpc_interfaces__msg__CrossCovariance2));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = psbmpc_interfaces__msg__CrossCovariance2__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        psbmpc_interfaces__msg__CrossCovariance2__fini(&data[i - 1]);
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
psbmpc_interfaces__msg__CrossCovariance2__Sequence__fini(psbmpc_interfaces__msg__CrossCovariance2__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      psbmpc_interfaces__msg__CrossCovariance2__fini(&array->data[i]);
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

psbmpc_interfaces__msg__CrossCovariance2__Sequence *
psbmpc_interfaces__msg__CrossCovariance2__Sequence__create(size_t size)
{
  psbmpc_interfaces__msg__CrossCovariance2__Sequence * array = (psbmpc_interfaces__msg__CrossCovariance2__Sequence *)malloc(sizeof(psbmpc_interfaces__msg__CrossCovariance2__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = psbmpc_interfaces__msg__CrossCovariance2__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
psbmpc_interfaces__msg__CrossCovariance2__Sequence__destroy(psbmpc_interfaces__msg__CrossCovariance2__Sequence * array)
{
  if (array) {
    psbmpc_interfaces__msg__CrossCovariance2__Sequence__fini(array);
  }
  free(array);
}
