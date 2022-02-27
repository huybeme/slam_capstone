// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from capstone_interfaces:msg/TB3Status.idl
// generated code does not contain a copyright notice

#ifndef CAPSTONE_INTERFACES__MSG__DETAIL__TB3_STATUS__FUNCTIONS_H_
#define CAPSTONE_INTERFACES__MSG__DETAIL__TB3_STATUS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "capstone_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "capstone_interfaces/msg/detail/tb3_status__struct.h"

/// Initialize msg/TB3Status message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * capstone_interfaces__msg__TB3Status
 * )) before or use
 * capstone_interfaces__msg__TB3Status__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_capstone_interfaces
bool
capstone_interfaces__msg__TB3Status__init(capstone_interfaces__msg__TB3Status * msg);

/// Finalize msg/TB3Status message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_capstone_interfaces
void
capstone_interfaces__msg__TB3Status__fini(capstone_interfaces__msg__TB3Status * msg);

/// Create msg/TB3Status message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * capstone_interfaces__msg__TB3Status__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_capstone_interfaces
capstone_interfaces__msg__TB3Status *
capstone_interfaces__msg__TB3Status__create();

/// Destroy msg/TB3Status message.
/**
 * It calls
 * capstone_interfaces__msg__TB3Status__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_capstone_interfaces
void
capstone_interfaces__msg__TB3Status__destroy(capstone_interfaces__msg__TB3Status * msg);


/// Initialize array of msg/TB3Status messages.
/**
 * It allocates the memory for the number of elements and calls
 * capstone_interfaces__msg__TB3Status__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_capstone_interfaces
bool
capstone_interfaces__msg__TB3Status__Sequence__init(capstone_interfaces__msg__TB3Status__Sequence * array, size_t size);

/// Finalize array of msg/TB3Status messages.
/**
 * It calls
 * capstone_interfaces__msg__TB3Status__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_capstone_interfaces
void
capstone_interfaces__msg__TB3Status__Sequence__fini(capstone_interfaces__msg__TB3Status__Sequence * array);

/// Create array of msg/TB3Status messages.
/**
 * It allocates the memory for the array and calls
 * capstone_interfaces__msg__TB3Status__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_capstone_interfaces
capstone_interfaces__msg__TB3Status__Sequence *
capstone_interfaces__msg__TB3Status__Sequence__create(size_t size);

/// Destroy array of msg/TB3Status messages.
/**
 * It calls
 * capstone_interfaces__msg__TB3Status__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_capstone_interfaces
void
capstone_interfaces__msg__TB3Status__Sequence__destroy(capstone_interfaces__msg__TB3Status__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // CAPSTONE_INTERFACES__MSG__DETAIL__TB3_STATUS__FUNCTIONS_H_
