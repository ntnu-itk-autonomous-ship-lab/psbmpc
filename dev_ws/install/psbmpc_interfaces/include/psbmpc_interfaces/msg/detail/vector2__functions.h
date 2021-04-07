// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from psbmpc_interfaces:msg/Vector2.idl
// generated code does not contain a copyright notice

#ifndef PSBMPC_INTERFACES__MSG__DETAIL__VECTOR2__FUNCTIONS_H_
#define PSBMPC_INTERFACES__MSG__DETAIL__VECTOR2__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "psbmpc_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "psbmpc_interfaces/msg/detail/vector2__struct.h"

/// Initialize msg/Vector2 message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * psbmpc_interfaces__msg__Vector2
 * )) before or use
 * psbmpc_interfaces__msg__Vector2__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_psbmpc_interfaces
bool
psbmpc_interfaces__msg__Vector2__init(psbmpc_interfaces__msg__Vector2 * msg);

/// Finalize msg/Vector2 message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_psbmpc_interfaces
void
psbmpc_interfaces__msg__Vector2__fini(psbmpc_interfaces__msg__Vector2 * msg);

/// Create msg/Vector2 message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * psbmpc_interfaces__msg__Vector2__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_psbmpc_interfaces
psbmpc_interfaces__msg__Vector2 *
psbmpc_interfaces__msg__Vector2__create();

/// Destroy msg/Vector2 message.
/**
 * It calls
 * psbmpc_interfaces__msg__Vector2__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_psbmpc_interfaces
void
psbmpc_interfaces__msg__Vector2__destroy(psbmpc_interfaces__msg__Vector2 * msg);


/// Initialize array of msg/Vector2 messages.
/**
 * It allocates the memory for the number of elements and calls
 * psbmpc_interfaces__msg__Vector2__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_psbmpc_interfaces
bool
psbmpc_interfaces__msg__Vector2__Sequence__init(psbmpc_interfaces__msg__Vector2__Sequence * array, size_t size);

/// Finalize array of msg/Vector2 messages.
/**
 * It calls
 * psbmpc_interfaces__msg__Vector2__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_psbmpc_interfaces
void
psbmpc_interfaces__msg__Vector2__Sequence__fini(psbmpc_interfaces__msg__Vector2__Sequence * array);

/// Create array of msg/Vector2 messages.
/**
 * It allocates the memory for the array and calls
 * psbmpc_interfaces__msg__Vector2__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_psbmpc_interfaces
psbmpc_interfaces__msg__Vector2__Sequence *
psbmpc_interfaces__msg__Vector2__Sequence__create(size_t size);

/// Destroy array of msg/Vector2 messages.
/**
 * It calls
 * psbmpc_interfaces__msg__Vector2__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_psbmpc_interfaces
void
psbmpc_interfaces__msg__Vector2__Sequence__destroy(psbmpc_interfaces__msg__Vector2__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // PSBMPC_INTERFACES__MSG__DETAIL__VECTOR2__FUNCTIONS_H_
