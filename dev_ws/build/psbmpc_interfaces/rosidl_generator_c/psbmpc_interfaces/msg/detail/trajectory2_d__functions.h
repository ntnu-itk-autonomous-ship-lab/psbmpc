// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from psbmpc_interfaces:msg/Trajectory2D.idl
// generated code does not contain a copyright notice

#ifndef PSBMPC_INTERFACES__MSG__DETAIL__TRAJECTORY2_D__FUNCTIONS_H_
#define PSBMPC_INTERFACES__MSG__DETAIL__TRAJECTORY2_D__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "psbmpc_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "psbmpc_interfaces/msg/detail/trajectory2_d__struct.h"

/// Initialize msg/Trajectory2D message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * psbmpc_interfaces__msg__Trajectory2D
 * )) before or use
 * psbmpc_interfaces__msg__Trajectory2D__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_psbmpc_interfaces
bool
psbmpc_interfaces__msg__Trajectory2D__init(psbmpc_interfaces__msg__Trajectory2D * msg);

/// Finalize msg/Trajectory2D message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_psbmpc_interfaces
void
psbmpc_interfaces__msg__Trajectory2D__fini(psbmpc_interfaces__msg__Trajectory2D * msg);

/// Create msg/Trajectory2D message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * psbmpc_interfaces__msg__Trajectory2D__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_psbmpc_interfaces
psbmpc_interfaces__msg__Trajectory2D *
psbmpc_interfaces__msg__Trajectory2D__create();

/// Destroy msg/Trajectory2D message.
/**
 * It calls
 * psbmpc_interfaces__msg__Trajectory2D__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_psbmpc_interfaces
void
psbmpc_interfaces__msg__Trajectory2D__destroy(psbmpc_interfaces__msg__Trajectory2D * msg);


/// Initialize array of msg/Trajectory2D messages.
/**
 * It allocates the memory for the number of elements and calls
 * psbmpc_interfaces__msg__Trajectory2D__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_psbmpc_interfaces
bool
psbmpc_interfaces__msg__Trajectory2D__Sequence__init(psbmpc_interfaces__msg__Trajectory2D__Sequence * array, size_t size);

/// Finalize array of msg/Trajectory2D messages.
/**
 * It calls
 * psbmpc_interfaces__msg__Trajectory2D__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_psbmpc_interfaces
void
psbmpc_interfaces__msg__Trajectory2D__Sequence__fini(psbmpc_interfaces__msg__Trajectory2D__Sequence * array);

/// Create array of msg/Trajectory2D messages.
/**
 * It allocates the memory for the array and calls
 * psbmpc_interfaces__msg__Trajectory2D__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_psbmpc_interfaces
psbmpc_interfaces__msg__Trajectory2D__Sequence *
psbmpc_interfaces__msg__Trajectory2D__Sequence__create(size_t size);

/// Destroy array of msg/Trajectory2D messages.
/**
 * It calls
 * psbmpc_interfaces__msg__Trajectory2D__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_psbmpc_interfaces
void
psbmpc_interfaces__msg__Trajectory2D__Sequence__destroy(psbmpc_interfaces__msg__Trajectory2D__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // PSBMPC_INTERFACES__MSG__DETAIL__TRAJECTORY2_D__FUNCTIONS_H_
