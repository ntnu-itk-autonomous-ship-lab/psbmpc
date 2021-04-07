// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from psbmpc_interfaces:msg/KinematicEstimateStamped.idl
// generated code does not contain a copyright notice

#ifndef PSBMPC_INTERFACES__MSG__DETAIL__KINEMATIC_ESTIMATE_STAMPED__FUNCTIONS_H_
#define PSBMPC_INTERFACES__MSG__DETAIL__KINEMATIC_ESTIMATE_STAMPED__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "psbmpc_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "psbmpc_interfaces/msg/detail/kinematic_estimate_stamped__struct.h"

/// Initialize msg/KinematicEstimateStamped message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * psbmpc_interfaces__msg__KinematicEstimateStamped
 * )) before or use
 * psbmpc_interfaces__msg__KinematicEstimateStamped__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_psbmpc_interfaces
bool
psbmpc_interfaces__msg__KinematicEstimateStamped__init(psbmpc_interfaces__msg__KinematicEstimateStamped * msg);

/// Finalize msg/KinematicEstimateStamped message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_psbmpc_interfaces
void
psbmpc_interfaces__msg__KinematicEstimateStamped__fini(psbmpc_interfaces__msg__KinematicEstimateStamped * msg);

/// Create msg/KinematicEstimateStamped message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * psbmpc_interfaces__msg__KinematicEstimateStamped__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_psbmpc_interfaces
psbmpc_interfaces__msg__KinematicEstimateStamped *
psbmpc_interfaces__msg__KinematicEstimateStamped__create();

/// Destroy msg/KinematicEstimateStamped message.
/**
 * It calls
 * psbmpc_interfaces__msg__KinematicEstimateStamped__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_psbmpc_interfaces
void
psbmpc_interfaces__msg__KinematicEstimateStamped__destroy(psbmpc_interfaces__msg__KinematicEstimateStamped * msg);


/// Initialize array of msg/KinematicEstimateStamped messages.
/**
 * It allocates the memory for the number of elements and calls
 * psbmpc_interfaces__msg__KinematicEstimateStamped__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_psbmpc_interfaces
bool
psbmpc_interfaces__msg__KinematicEstimateStamped__Sequence__init(psbmpc_interfaces__msg__KinematicEstimateStamped__Sequence * array, size_t size);

/// Finalize array of msg/KinematicEstimateStamped messages.
/**
 * It calls
 * psbmpc_interfaces__msg__KinematicEstimateStamped__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_psbmpc_interfaces
void
psbmpc_interfaces__msg__KinematicEstimateStamped__Sequence__fini(psbmpc_interfaces__msg__KinematicEstimateStamped__Sequence * array);

/// Create array of msg/KinematicEstimateStamped messages.
/**
 * It allocates the memory for the array and calls
 * psbmpc_interfaces__msg__KinematicEstimateStamped__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_psbmpc_interfaces
psbmpc_interfaces__msg__KinematicEstimateStamped__Sequence *
psbmpc_interfaces__msg__KinematicEstimateStamped__Sequence__create(size_t size);

/// Destroy array of msg/KinematicEstimateStamped messages.
/**
 * It calls
 * psbmpc_interfaces__msg__KinematicEstimateStamped__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_psbmpc_interfaces
void
psbmpc_interfaces__msg__KinematicEstimateStamped__Sequence__destroy(psbmpc_interfaces__msg__KinematicEstimateStamped__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // PSBMPC_INTERFACES__MSG__DETAIL__KINEMATIC_ESTIMATE_STAMPED__FUNCTIONS_H_
